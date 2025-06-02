#include <Arduino.h>
#include <Wire.h>                         // I2C library
#include <Protocentral_FDC1004.h>         // FDC library

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// Display headers - Kept for compilation if lv_conf.h is included, but LVGL is disabled
#include <lvgl.h>
#include "Arduino_GFX_Library.h" // Not strictly needed if GFX object is not created
#include "lv_conf.h"

#include "pin_config.h"    // Ensure IIC_SDA, IIC_SCL, LCD_BL are defined
#include "SensorQMI8658.hpp"

// USB Communication header
#include "HWCDC.h"

HWCDC USBSerial;

#define SYS_EN 41
#define SYS_OUT 40

// FDC Defines
#define UPPER_BOUND  0X4000
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define FDC_CHANNEL    0
#define FDC_MEASURMENT 0

SensorQMI8658 qmi;
IMUdata acc; 
// SemaphoreHandle_t lvgl_mutex; // Not needed if LVGL is fully disabled

typedef struct {
    uint32_t sync_time;
} sync_packet_t;

// MAC address of the MASTER RECEIVER (receiver-sync.txt)
uint8_t masterReceiverMacAddress[] = {0x80, 0x7D, 0x3A, 0xF3, 0xBF, 0x4C}; // CORRECTED NAME

esp_now_peer_info_t masterPeerInfo_watch;
const uint8_t WIFI_CHANNEL_WATCH = 6;

QueueHandle_t sensorDataQueue_watch;

TaskHandle_t FdcDataCollectionTaskHandle = NULL;
TaskHandle_t WatchEspNowSendTaskHandle = NULL; 

int capdac = 0;

volatile uint32_t watch_current_synced_timestamp = 0;
volatile bool watch_time_synced_at_least_once = false;
unsigned long watch_last_sync_ref_millis = 0;
volatile uint32_t watch_sync_event_marker = 0;

// GFX objects - Not created if display is off
// Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);
// Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST, 0, true, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

static uint8_t watch_system_reboot_count = 0; 
void watch_trigger_reboot(void *arg) {
  watch_system_reboot_count++;
  if (watch_system_reboot_count >= 300) { 
    USBSerial.println("Watch: Reboot counter. Restarting."); ESP.restart(); 
    }
}

void watch_on_sync_data_recv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(sync_packet_t)) {
        sync_packet_t syncData;
        memcpy(&syncData, incomingData, sizeof(syncData));
        if (syncData.sync_time == 0) {
            watch_current_synced_timestamp = 0;
            watch_last_sync_ref_millis = millis();
            watch_time_synced_at_least_once = false;
            watch_sync_event_marker = 0;
            USBSerial.println("Watch: RESET sync received.");
        } else {
            watch_current_synced_timestamp = syncData.sync_time;
            watch_last_sync_ref_millis = millis();
            watch_time_synced_at_least_once = true;
            watch_sync_event_marker = 1;
        }
    }
}

void FdcDataCollectionTask(void *pvParameters) {
  FDC1004 FDC_sensor;
  char data_string_buffer[60];
  int buffer_write_offset;

  USBSerial.println("Watch: FDC Task (Display OFF, 200SPS FDC, ~100Hz Send).");

  while (1) {
    buffer_write_offset = 0;
    if(digitalRead(SYS_OUT) == 0) { digitalWrite(SYS_EN, LOW); }
    
    int32_t capacitance_tx_femtofarads;

    FDC_sensor.configureMeasurementSingle(FDC_MEASURMENT, FDC_CHANNEL, capdac);
    FDC_sensor.triggerSingleMeasurement(FDC_MEASURMENT, FDC1004_200HZ); 
    vTaskDelay(pdMS_TO_TICKS(6)); 
    
    uint16_t fdc_raw[2];
    if (!FDC_sensor.readMeasurement(FDC_MEASURMENT, fdc_raw)) {
      int16_t msb = (int16_t)fdc_raw[0];
      capacitance_tx_femtofarads = (((int32_t)457 * msb) / 1000) + ((int32_t)3028 * capdac);
      if (watch_sync_event_marker == 1) { capacitance_tx_femtofarads = 25000; watch_sync_event_marker = 0; }
      if (msb > UPPER_BOUND && capdac < FDC1004_CAPDAC_MAX) capdac++;
      else if (msb < LOWER_BOUND && capdac > 0) capdac--;
    } else { 
      capacitance_tx_femtofarads = 0;
    }
 
    unsigned long current_capture_ms = millis();
    uint32_t timestamp_tx;
    if (watch_time_synced_at_least_once) {
        timestamp_tx = watch_current_synced_timestamp + (current_capture_ms - watch_last_sync_ref_millis);
    } else { 
        timestamp_tx = (watch_current_synced_timestamp == 0 && !watch_time_synced_at_least_once) ? 0 : watch_current_synced_timestamp + (current_capture_ms - watch_last_sync_ref_millis);
    }

    buffer_write_offset += snprintf(data_string_buffer + buffer_write_offset, sizeof(data_string_buffer) - buffer_write_offset, "C%05ld", capacitance_tx_femtofarads);
    buffer_write_offset += snprintf(data_string_buffer + buffer_write_offset, sizeof(data_string_buffer) - buffer_write_offset, "T%08lu\n", timestamp_tx);
    
    if (buffer_write_offset > 0) {
        if (xQueueSend(sensorDataQueue_watch, (void *)data_string_buffer, pdMS_TO_TICKS(2)) != pdTRUE) {
            // USBSerial.println("Watch: FDC Queue full!");
        }
    }
    vTaskDelay(pdMS_TO_TICKS(3)); // Approx 7ms FDC wait + 1ms proc + 2ms delay = ~10ms loop (100Hz)
  }
}

void WatchEspNowSendTask(void *pvParameters) {
  char send_buf[60]; 
  USBSerial.println("Watch: ESP-NOW Send Task started.");
  while (true) {
    if (xQueueReceive(sensorDataQueue_watch, &send_buf, portMAX_DELAY)) {
        if (watch_time_synced_at_least_once || watch_current_synced_timestamp != 0) {
            // Using the consistent 'masterReceiverMacAddress'
            esp_now_send(masterReceiverMacAddress, (uint8_t *)send_buf, strlen(send_buf));
        }
    }
  }
}

void setup() {
  pinMode(SYS_EN, OUTPUT); digitalWrite(SYS_EN, HIGH); 
  pinMode(SYS_OUT, INPUT);    
  USBSerial.begin(115200); 
  unsigned long t_start = millis(); while(!USBSerial && (millis()-t_start < 3000)) delay(10);
  USBSerial.println("\n--- FDC1004 Watch (ESP-NOW Sync - v10 MAC Fix, Display OFF) ---");

  Wire.begin(IIC_SDA, IIC_SCL); USBSerial.println("Watch: I2C Initialized.");

  WiFi.mode(WIFI_STA); delay(100); 
  USBSerial.print("Watch MAC: "); USBSerial.println(WiFi.macAddress()); 

  esp_err_t ch_err = esp_wifi_set_channel(WIFI_CHANNEL_WATCH, WIFI_SECOND_CHAN_NONE);
  if (ch_err == ESP_OK) { USBSerial.printf("Watch: WiFi channel set to %d\n", WIFI_CHANNEL_WATCH); }
  else { USBSerial.printf("Watch: Error setting WiFi channel: %s\n", esp_err_to_name(ch_err)); }
  
  if (esp_now_init() != ESP_OK) { ESP.restart(); }
  USBSerial.println("Watch: ESP-NOW Initialized.");
  esp_now_register_recv_cb(watch_on_sync_data_recv); 

  memset(&masterPeerInfo_watch, 0, sizeof(masterPeerInfo_watch)); 
  // Using the consistent 'masterReceiverMacAddress'
  memcpy(masterPeerInfo_watch.peer_addr, masterReceiverMacAddress, 6);
  masterPeerInfo_watch.channel = WIFI_CHANNEL_WATCH; 
  masterPeerInfo_watch.encrypt = false;
  if (esp_now_add_peer(&masterPeerInfo_watch) != ESP_OK) { USBSerial.println("Watch: Failed to add master peer."); }
  else { USBSerial.println("Watch: Master peer added."); }
  watch_last_sync_ref_millis = millis(); 
  
  // --- LVGL DISPLAY DISABLED ---
  USBSerial.println("Watch: LVGL Display is DISABLED.");
  pinMode(LCD_BL, OUTPUT);    // LCD_BL is defined in pin_config.h
  digitalWrite(LCD_BL, LOW);  // Turn Backlight OFF (assuming LOW is OFF)
  pinMode(38, OUTPUT); digitalWrite(38, LOW); // Keep as per your original setup if it controls display power
  // --- End LVGL Display Disabled ---

  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS)) { USBSerial.println("Watch: QMI8658 init failed."); }
  else {
    USBSerial.println("Watch: QMI8658 Initialized.");
    qmi.configAccelerometer(SensorQMI8658::ACC_RANGE_4G, SensorQMI8658::ACC_ODR_1000Hz, SensorQMI8658::LPF_MODE_0);
    qmi.enableAccelerometer();
  }
  
  sensorDataQueue_watch = xQueueCreate(15, sizeof(char) * 60);  
  if (!sensorDataQueue_watch) { USBSerial.println("Watch: Error creating queue. Halting."); while (1); }
  USBSerial.println("Watch: Sensor data queue created.");

  xTaskCreatePinnedToCore(FdcDataCollectionTask, "FDC_Task", 4096, NULL, 3, &FdcDataCollectionTaskHandle, 0); 
  xTaskCreatePinnedToCore(WatchEspNowSendTask, "NOW_Send_Task", 4096, NULL, 2, &WatchEspNowSendTaskHandle, 0); 
  // LvglDisplayTask is not created

  USBSerial.println("Watch: All setup complete.");
}

void loop() {
  vTaskDelay(pdMS_TO_TICKS(100)); 
}