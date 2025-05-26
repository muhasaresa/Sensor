#include <Arduino.h>
#include <Wire.h>                         // I2C library
#include <Protocentral_FDC1004.h>         // FDC library


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Include this for esp_wifi_set_max_tx_power

// Display headers
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "lv_conf.h"


#include "pin_config.h"
#include "SensorQMI8658.hpp"

// USB Communication header
#include "HWCDC.h"

HWCDC USBSerial;

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2



#define SYS_EN 41  // this pin is used to keep the board powered on battery
#define SYS_OUT 40

// Defines for LVGL
#define GRAPH_ULIMIT 15
#define GRAPH_LLIMIT -1
#define FDC_VALUE_SCALER 1

// #defines for FDC
#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL 0                          // channel to be read
#define MEASURMENT 0                       // measurment channel

static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

SensorQMI8658 qmi;
IMUdata acc;
SemaphoreHandle_t lvgl_mutex;
//SemaphoreHandle_t fdc_mutex;

lv_obj_t *label;               // Global label object
lv_obj_t *chart;               // Global chart object
lv_chart_series_t *fdc_series; // fdc_series

// MAC address of the receiver (replace with your receiver’s MAC address)
uint8_t receiverAddress[] = {0x80, 0x7D, 0x3A, 0xF3, 0xBF, 0x4C};  // Example MAC; update it for your receiver
// FreeRTOS queue to pass data between tasks
QueueHandle_t sensorDataQueue;

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t gfxTaskHandle = NULL;
TaskHandle_t nowTaskHandle = NULL;



int capdac = 0;
char result[100];
const int maxCapDAC = 31;       // Maximum CAPDAC value
const float targetBaseline = 0; // Target baseline capacitance
const float tolerance = 1.0;    // Tolerance range around baseline
const int numSamples = 1000;    // Number of samples for averaging


unsigned long previousMillis = 0; 

Arduino_DataBus *bus = new Arduino_ESP32SPI(LCD_DC, LCD_CS, LCD_SCK, LCD_MOSI);

Arduino_GFX *gfx = new Arduino_ST7789(bus, LCD_RST /* RST */,
                                      0 /* rotation */, true /* IPS */, LCD_WIDTH, LCD_HEIGHT, 0, 20, 0, 0);

#if LV_USE_LOG != 0
/* Serial debugging */
void my_print(const char *buf)
{
  USBSerial.printf(buf);
  USBSerial.flush();
}
#endif


/* Display flushing */
void my_disp_flush(lv_disp_drv_t *disp, const lv_area_t *area, lv_color_t *color_p)
{
  uint32_t w = (area->x2 - area->x1 + 1);
  uint32_t h = (area->y2 - area->y1 + 1);

#if (LV_COLOR_16_SWAP != 0)
  gfx->draw16bitBeRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#else
  gfx->draw16bitRGBBitmap(area->x1, area->y1, (uint16_t *)&color_p->full, w, h);
#endif

  lv_disp_flush_ready(disp);
}

void example_increase_lvgl_tick(void *arg)
{
  /* Tell LVGL how many milliseconds has elapsed */
  lv_tick_inc(EXAMPLE_LVGL_TICK_PERIOD_MS);
}

static uint8_t count = 0;
void example_increase_reboot(void *arg)
{
  count++;
  if (count == 30)
  {
    esp_restart();
  }
}
// Callback when data is sent
//void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    //Serial.print("Last Packet Send Status: ");
    //Serial.println(status == ESP_NOW_SEND_SUCCESS ? "Delivery Success" : "Delivery Fail");
//}



// Function that will be executed as a task
void Task1(void *pvParameters) {
  FDC1004 FDC;
  uint32_t ts = 0;
  char buffer[200];
  int offset = 0;
  float fdc_capacitance = 0.0;
  //USBSerial.println("Task 1");
  while (1) {
    if(digitalRead(SYS_OUT) == 0){ // This if statement turns off the device on pressing power button
      digitalWrite(SYS_EN, LOW);
    }
    // Serial.print("ts: " + String(ts));
    TickType_t startTick = xTaskGetTickCount();

 
  
    FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
    FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ);
    
    uint16_t value[2];
    vTaskDelay(pdMS_TO_TICKS(7)); // Wait for 7 ms
    if (! FDC.readMeasurement(MEASURMENT, value))
    {
      int16_t msb = (int16_t) value[0];
      int32_t capacitance = ((int32_t)457) * ((int32_t)msb); //in attofarads
      capacitance /= 1000;   //in femtofarads
      capacitance += ((int32_t)3028) * ((int32_t)capdac);
      //Serial.print("C");
      //Serial.println(capacitance);
      //Serial.println((((float)capacitance/1000)),4);
      fdc_capacitance = (((float)capacitance / 1000));
      offset += snprintf(buffer+offset,sizeof(buffer),"C%05d",capacitance);
      //USBSerial.println(capacitance);
      if (msb > UPPER_BOUND)               // adjust capdac accordingly
      {
        if (capdac < FDC1004_CAPDAC_MAX)
          capdac++;
      }
      else if (msb < LOWER_BOUND)
      {
        if (capdac > 0)
          capdac--;
      }
    }
 
 /* code */
    if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z))
    {
      offset  += snprintf(buffer+offset,sizeof(buffer), "X%+02.2fY%+02.2f%Z%+02.2f",acc.x, acc.y, acc.z);

      //USBSerial.print("{ACCEL: ");
      //USBSerial.print(acc_text);
      //USBSerial.println("}");
    }
   // if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY))
   // {
      // USBSerial.println("Inside FDC Task");
   //   lv_chart_set_next_value(chart, fdc_series, fdc_capacitance * FDC_VALUE_SCALER);
  //    lv_label_set_text(label, buffer);
  //    xSemaphoreGive(lvgl_mutex);
  //  }

    //String acc_data = "{" + String(acc.x) + "," + String(acc.y) + "," + String(acc.z) + "}";

    
    ts++;
    TickType_t endTick = xTaskGetTickCount();
    // Calculate the elapsed ticks
    TickType_t elapsedTicks = endTick - startTick;

    // Convert ticks to milliseconds
    uint32_t elapsedTimeMs = elapsedTicks * (1000 / configTICK_RATE_HZ);
    previousMillis = previousMillis + elapsedTimeMs;
    offset += snprintf(buffer+offset,sizeof(buffer),"T%07d\n",previousMillis);
    
            // Send the data to the BLE task through the queue
    xQueueSend(sensorDataQueue, (void *)buffer, portMAX_DELAY);
    //Serial.print(buffer);
    //Serial.println("Time taken: " + String(elapsedTimeMs) );

    // Reset the buffer after sending
    memset(buffer, 0, sizeof(buffer));
    offset = 0;
  }
}

void gfxTask(void *pvParameters){
  while (true)
  {
    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    lv_timer_handler();
    xSemaphoreGive(lvgl_mutex);
    vTaskDelay(70 / portTICK_PERIOD_MS);
    //USBSerial.println("Task gfx");
  }

}

void nowTask(void * pvParameters) {
  char txbuffer[200];
  //USBSerial.println("Task now");
  while (true) {
        // Wait until both Task 1 and Task 2 have finished sampling
        //Serial.print("Buffer");
        
      // Wait to receive sensor data from the analogReadTask
        if (xQueueReceive(sensorDataQueue, &txbuffer, portMAX_DELAY)) {
            // Set the BLE characteristic value with the received packet
            // Send data over ESP-NOW
            esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)txbuffer, sizeof(txbuffer));
            //USBSerial.print(txbuffer);
                    // Wait a short period to avoid flooding the receiver
            memset(txbuffer, 0, sizeof(txbuffer));
            //vTaskDelay(50 / portTICK_PERIOD_MS);
                    // Clear the buffer after sending
        


        }
    }
}



void setup() {

// put your setup code here, to run once:
  pinMode(SYS_EN, OUTPUT);
  digitalWrite(SYS_EN, HIGH); // Setting this pin to high enables the device to be continuously powered by the battery

  pinMode(SYS_OUT, INPUT);
  // Initialize serial communication
  USBSerial.begin(115200); /* prepare for possible serial debug */

  lvgl_mutex = xSemaphoreCreateMutex();

  gfx->begin();
  pinMode(15, OUTPUT); // LCD BL
  pinMode(38, OUTPUT);
  //pinMode(5, OUTPUT);
  digitalWrite(15, LOW);
  digitalWrite(38, HIGH);
  String LVGL_Arduino = "Hello Arduino! ";
  LVGL_Arduino += String('V') + lv_version_major() + "." + lv_version_minor() + "." + lv_version_patch();

  USBSerial.println(LVGL_Arduino);
  USBSerial.println("start setup");
  lv_init();

#if LV_USE_LOG != 0
  lv_log_register_print_cb(my_print); /* register print function for debugging */
#endif

  lv_disp_draw_buf_init(&draw_buf, buf, NULL, LCD_WIDTH * LCD_HEIGHT / 10);

  /*Initialize the display*/
  static lv_disp_drv_t disp_drv;
  lv_disp_drv_init(&disp_drv);
  /*Change the following line to your display resolution*/
  disp_drv.hor_res = LCD_WIDTH;
  disp_drv.ver_res = LCD_HEIGHT;
  disp_drv.flush_cb = my_disp_flush;
  disp_drv.draw_buf = &draw_buf;
  lv_disp_drv_register(&disp_drv);

  /*Initialize the (dummy) input device driver*/
  static lv_indev_drv_t indev_drv;
  lv_indev_drv_init(&indev_drv);
  indev_drv.type = LV_INDEV_TYPE_POINTER;
  lv_indev_drv_register(&indev_drv);

  label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "COERs");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);

  const esp_timer_create_args_t lvgl_tick_timer_args = {
      .callback = &example_increase_lvgl_tick,
      .name = "lvgl_tick"};

  const esp_timer_create_args_t reboot_timer_args = {
      .callback = &example_increase_reboot,
      .name = "reboot"};

  esp_timer_handle_t lvgl_tick_timer = NULL;
  esp_timer_create(&lvgl_tick_timer_args, &lvgl_tick_timer);
  esp_timer_start_periodic(lvgl_tick_timer, EXAMPLE_LVGL_TICK_PERIOD_MS * 1000);

  lv_timer_handler(); /* let the GUI do its work */
  delay(3000);

  // Delete the first label
  lv_obj_del(label);

  /* Create chart */
  chart = lv_chart_create(lv_scr_act());
  lv_obj_set_size(chart, 240, 280);                                               // Set the size of the chart
  lv_obj_align(chart, LV_ALIGN_CENTER, 0, 0);                                     // Center the chart
  lv_chart_set_type(chart, LV_CHART_TYPE_LINE);                                   // Set the chart type to line
  lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, GRAPH_LLIMIT, GRAPH_ULIMIT); // Set the Y-axis range
  lv_chart_set_point_count(chart, 20);                                            // Set the number of data points

  // Add a series for the chart
  fdc_series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_RED), LV_CHART_AXIS_PRIMARY_Y);

  // Create a label to display accelerometer data
  label = lv_label_create(lv_scr_act());
  lv_label_set_text(label, "X: 0.00 Y: 0.00 Z: 0.00");

  // Style the label
  lv_obj_set_style_text_color(label, lv_palette_main(LV_PALETTE_RED), LV_PART_MAIN); // Match the label text color with the chart series color
  lv_obj_set_style_text_font(label, &lv_font_montserrat_14, LV_PART_MAIN);           // Use a modern font
  lv_obj_set_style_bg_opa(label, LV_OPA_TRANSP, LV_PART_MAIN);                       // Make the background transparent

  // Position the label inside the chart
  // lv_obj_align(label, LV_ALIGN_TOP_LEFT, 10, 10); // Offset slightly from the top-left corner
  lv_obj_align(label, LV_ALIGN_TOP_MID, 0, +5); // Offset upwards by 10 pixels from the bottom edge

  USBSerial.println("gfx setup complete");

      // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
 
  //WiFi.disconnect(); // Disconnect from any network (not needed for ESP-NOW)
    // Select a WiFi channel. For example, choose channel 6.
  // The second parameter sets the secondary channel; here we use WIFI_SECOND_CHAN_NONE.
  uint8_t channel = 6;
  esp_err_t err = esp_wifi_set_channel(channel, WIFI_SECOND_CHAN_NONE);
  if (err == ESP_OK) {
    Serial.print("WiFi channel set to: ");
    Serial.println(channel);
  } else {
    Serial.print("Error setting WiFi channel: ");
    Serial.println(err);
  }
    // Initialize the Wi-Fi driver
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
        USBSerial.println("Error setting Wi-Fi mode");
        return;
    }
      // Set the maximum TX power (values: 8 - 84, in 0.25 dBm units)
    esp_wifi_set_max_tx_power(50); // Example: 20 corresponds to 5 dBm
   //Wire.begin();


 // Initialize the QMI8658
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
  {

    while (1)
    {
      USBSerial.println("in accelerometer");
      delay(1000);
    }
  }

  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0);

  qmi.enableAccelerometer();
  qmi.dumpCtrlRegister();
      // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        USBSerial.println("Error initializing ESP-NOW");
        ESP.restart();  // Restart if initialization fails
    }
    //esp_now_register_send_cb(onDataSent);

    // Register peer
    esp_now_peer_info_t peerInfo;
    memset(&peerInfo, 0, sizeof(peerInfo));  // Clear the structure
    memcpy(peerInfo.peer_addr, receiverAddress, 6);
    peerInfo.channel = 0;  
    peerInfo.encrypt = false;
    
    // Add the peer
    if (esp_now_add_peer(&peerInfo) != ESP_OK) {
        USBSerial.println("Failed to add peer");
        return;
    }
    else {
    USBSerial.println("Peer added successfully");
  }

     // Create the sensor data queue
    sensorDataQueue = xQueueCreate(10, sizeof(char) * 200);  // Queue to store 10 data packets

        // Check if the queue was created successfully
    if (sensorDataQueue == NULL) {
        USBSerial.println("Error creating the queue.");
        while (1);  // Halt the program
    }

    // Create Task 1 (Highest priority)
    xTaskCreate(Task1, "Task1", 6000, NULL, 3, &Task1Handle);

    // Create Task 2 (Lower priority) Enable if needed
    //xTaskCreatePinnedToCore(Task2, "Task2", 1000, NULL, 2, &Task2Handle, 1);

 
    xTaskCreate(nowTask, "Task2", 6000, NULL, 2, &nowTaskHandle);
   // Create Task 3 (Lowest priority)
    //xTaskCreate(gfxTask, "Task3", 6000, NULL, 1, &gfxTaskHandle);
    USBSerial.println("end setup");
    gfx->displayOff();
    digitalWrite(LCD_BL, LOW);


}

void loop() {
  // Nothing to do in the loop
}