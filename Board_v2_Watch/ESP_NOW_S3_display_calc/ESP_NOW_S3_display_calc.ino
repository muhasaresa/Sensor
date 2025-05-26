#include <Arduino.h>
#include <Wire.h> // I2C library
#include <arduinoFFT.h>

// FreeRTOS headers
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

// Display headers
#include <lvgl.h>
#include "Arduino_GFX_Library.h"
#include "lv_conf.h"
#include "Logo.h"
#include "pin_config.h"
#include "SensorQMI8658.hpp"

// FDC library
#include <Protocentral_FDC1004.h>

// BLE Headers
#include <BLEDevice.h>
#include <BLEServer.h>
#include <BLEUtils.h>
#include <BLE2902.h>

// USB Communication header
#include "HWCDC.h"

HWCDC USBSerial;

#define EXAMPLE_LVGL_TICK_PERIOD_MS 2


#define SYS_EN 41  // this pin is used to keep the board powered on battery
#define SYS_OUT 40

// Defines for LVGL
#define GRAPH_ULIMIT 30000
#define GRAPH_LLIMIT -1
#define FDC_VALUE_SCALER 1
#define POINT_COUNT  100

// #defines for FDC
#define UPPER_BOUND 0X4000 // max readout capacitance
#define LOWER_BOUND (-1 * UPPER_BOUND)
#define CHANNEL 0    // channel to be read
#define MEASURMENT 0 // measurment channel

#define BASELINE_WINDOW 50  // number of samples in the moving average

// ========== SETTINGS ==========
#define SAMPLES 128
#define FS 10

#define ACC_THRESHOLD 0.005

arduinoFFT FFT = arduinoFFT();
double vReal[SAMPLES];
double vImag[SAMPLES];
float accMag[SAMPLES];


int16_t baseline_buffer[BASELINE_WINDOW] = {0};
int baseline_index = 0;
int32_t baseline_sum = 0; // use 32-bit to avoid overflow

int16_t remove_baseline(int16_t new_sample) {
    // Subtract the oldest sample from the sum
    baseline_sum -= baseline_buffer[baseline_index];

    // Replace it with the new sample
    baseline_buffer[baseline_index] = new_sample;

    // Add the new sample to the sum
    baseline_sum += new_sample;

    // Move to the next index in circular buffer
    baseline_index = (baseline_index + 1) % BASELINE_WINDOW;

    // Compute average (baseline)
    int16_t avg = baseline_sum / BASELINE_WINDOW;

    // Return baseline-removed value
    return new_sample - avg;
}



static lv_disp_draw_buf_t draw_buf;
static lv_color_t buf[LCD_WIDTH * LCD_HEIGHT / 10];

SensorQMI8658 qmi;

IMUdata acc;

SemaphoreHandle_t lvgl_mutex;
SemaphoreHandle_t fdc_mutex;

lv_obj_t *label;               // Global label object
lv_obj_t *chart;               // Global chart object
lv_chart_series_t *fdc_series; // fdc_series

// Bluetooth global definitions start
#define SERVICE_UUID "6e400001-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_RX "6e400002-b5a3-f393-e0a9-e50e24dcca9e"
#define CHARACTERISTIC_UUID_TX "6e400003-b5a3-f393-e0a9-e50e24dcca9e"

BLECharacteristic *pTxCharacteristic;
bool deviceConnected = false;

// BLE Queues
QueueHandle_t sensorQueueFast, sensorQueueSlow;

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

// BLE Callbacks
class MyServerCallbacks : public BLEServerCallbacks
{
  void onConnect(BLEServer *pServer)
  {
    deviceConnected = true;
  }

  void onDisconnect(BLEServer *pServer)
  {
    deviceConnected = false;

    // Start advertising
    BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
    pAdvertising->addServiceUUID(SERVICE_UUID);
    pAdvertising->start();
  }
};

class MyCallbacks : public BLECharacteristicCallbacks
{
  void onWrite(BLECharacteristic *pCharacteristic)
  {
    String rxValue = pCharacteristic->getValue();
    if (rxValue.length() > 0)
    {
      // Process received data here
    }
  }
};



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

bool checkIfAtRest() {
  float sum = 0, sumSq = 0;
  for (int i = 0; i < SAMPLES; i++) {
    float mag = accMag[i];
    sum += mag;
    sumSq += mag * mag;
  }
  float mean = sum / SAMPLES;
  float stdDev = sqrt((sumSq / SAMPLES) - (mean * mean));
  return stdDev < ACC_THRESHOLD;
}

double calculateHR() {
  FFT.Windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.Compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.ComplexToMagnitude(vReal, vImag, SAMPLES);

//int low = (58.0 / 60.0 * SAMPLES) / FS;  // ≈ 0.97 Hz
//int high = (90.0 / 60.0 * SAMPLES) / FS; // ≈ 1.5 Hz

  int low = (0.1 * SAMPLES) / FS;   // ≈ index 2
  int high = (0.5 * SAMPLES) / FS;  // ≈ index 12

  double peak = 0;
  int peakIndex = 0;
  for (int i = low; i <= high; i++) {
    if (vReal[i] > peak) {
      peak = vReal[i];
      peakIndex = i;
    }
  }

  double freq = (peakIndex * FS) / (double)SAMPLES;
  return freq * 60.0;
}

void TaskSendData(void *parameter)
{
  String batchData;
  int itemsPerBatch = 2; // Number of data items to retrieve in each iteration
  String queueData;

  while (1)
  {
    if (true)
    {
      batchData = ""; // Clear batch data at the start of each iteration

      if (xQueueReceive(sensorQueueFast, &queueData, 0) == pdPASS)
      {
        batchData += queueData + "\n"; // Append data with a separator
      }

      // Try to get data from sensorQueueSlow
      if (xQueueReceive(sensorQueueSlow, &queueData, 0) == pdPASS)
      {
        batchData += queueData + "\n";
      }
      // }

      // Send the batched data as a single notification if there’s any data
      if (!batchData.isEmpty())
      {
        Serial.println("Data: " + batchData);
        pTxCharacteristic->setValue(batchData);
        pTxCharacteristic->notify();
      }
    }

    vTaskDelay(20 / portTICK_PERIOD_MS); // Delay for 20 ms to control notification rate
  }
}

void fdc_task(void *pvParameters)
{
  String data;
  int packetNum = 0;
  char acc_text[50];
  FDC1004 FDC;
  int capdac = 0;
  float fdc_capacitance = 0.0;
  int fdc_graph_value = 0;
  int16_t disCapacitance = 0;
  int sampleno = 0;

  while (true)
  {
    data = "";
    data += String(packetNum);
    data += ", ";

    if(digitalRead(SYS_OUT) == 0){ // This if statement turns off the device on pressing power button
      digitalWrite(SYS_EN, LOW);
    }

    USBSerial.print(String(packetNum));
    USBSerial.print(", ");

    FDC.configureMeasurementSingle(MEASURMENT, CHANNEL, capdac);
    FDC.triggerSingleMeasurement(MEASURMENT, FDC1004_400HZ);

    uint16_t value[2];

    vTaskDelay(20 / portTICK_PERIOD_MS);

    if (!FDC.readMeasurement(MEASURMENT, value))
    {
      int16_t msb = (int16_t)value[0];
      int32_t capacitance = ((int32_t)457) * ((int32_t)msb); // in attofarads
      capacitance /= 1000;                                   // in femtofarads
      capacitance += ((int32_t)3028) * ((int32_t)capdac);

      fdc_capacitance = (((float)capacitance / 10000));
      //USBSerial.print("fdc_capacitance");
      //USBSerial.println(fdc_capacitance, 4);
      disCapacitance = int16_t(remove_baseline(capacitance));
     
      //disCapacitance = int16_t(highpass_filter(capacitance));
      data += String(fdc_capacitance, 4);

      if (msb > UPPER_BOUND) // adjust capdac accordingly
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

    xQueueSend(sensorQueueFast, &data, portMAX_DELAY);
    packetNum++;
    if (qmi.getDataReady() && qmi.getAccelerometer(acc.x, acc.y, acc.z))
    {
      accMag[sampleno] = sqrt(acc.x * acc.x + acc.y * acc.y + acc.z * acc.z);

      
      }

      vReal[sampleno] = disCapacitance;
      vImag[sampleno] = 0;
      sampleno = sampleno + 1;
    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY))
    {
      // USBSerial.println("Inside FDC Task");
          // Shift data to the left
    for (int i = 1; i < POINT_COUNT; i++) {
        fdc_series->y_points[i - 1] = fdc_series->y_points[i];
    }
      fdc_series->y_points[POINT_COUNT - 1] = disCapacitance;
      // Autoscale Y axis
    int16_t min_y = INT16_MAX, max_y = INT16_MIN;
    for (int i = 0; i < POINT_COUNT; i++) {
        if (fdc_series->y_points[i] < min_y) min_y = fdc_series->y_points[i];
        if (fdc_series->y_points[i] > max_y) max_y = fdc_series->y_points[i];
    }

    lv_chart_set_range(chart, LV_CHART_AXIS_PRIMARY_Y, min_y - 2, max_y + 2);

    // Refresh the chart
    lv_chart_refresh(chart);
      //lv_chart_set_next_value(chart, fdc_series, disCapacitance * FDC_VALUE_SCALER);
      //lv_label_set_text(label, acc_text);
      xSemaphoreGive(lvgl_mutex);
    }
    
    if (sampleno >= SAMPLES){
    sampleno = 0;}

    vTaskDelay(80 / portTICK_PERIOD_MS);
  }
}

void accelerometer_task(void *pvParameters)
{
  char acc_text[50];
  while (true)
  {
    /* code */
    if (checkIfAtRest())
    {
      int hr = round(calculateHR());
     if (hr < 3 || hr > 40)
      {
        snprintf(acc_text, sizeof(acc_text), "BR: __"); 
      }
      else{
      snprintf(acc_text, sizeof(acc_text), "BR: %d",hr); }


    }
    else{
      snprintf(acc_text, sizeof(acc_text), "Motion detected");
    }
      memset(vReal, 0, sizeof(vReal));  // Set all bytes to 0
      memset(vImag, 0, sizeof(vImag));  // Set all bytes to 0
    String acc_data = "{" + String(acc.x) + "," + String(acc.y) + "," + String(acc.z) + "}";

    if (xSemaphoreTake(lvgl_mutex, portMAX_DELAY))
    {
      // Perform LVGL operations
      lv_label_set_text(label, acc_text);

      xSemaphoreGive(lvgl_mutex);
    }
    xQueueSend(sensorQueueSlow, &acc_data, portMAX_DELAY);
    vTaskDelay(13000 / portTICK_PERIOD_MS);
  }
}

void gui_task(void *pvParameters)
{
  while (true)
  {
    xSemaphoreTake(lvgl_mutex, portMAX_DELAY);
    lv_timer_handler();
    xSemaphoreGive(lvgl_mutex);
    vTaskDelay(5 / portTICK_PERIOD_MS);
  }
}

void setup()
{
  // put your setup code here, to run once:
  pinMode(SYS_EN, OUTPUT);
  digitalWrite(SYS_EN, HIGH); // Setting this pin to high enables the device to be continuously powered by the battery

  pinMode(SYS_OUT, INPUT);

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

  lv_init();

#if LV_USE_LOG != 0
  //lv_log_register_print_cb(my_print); /* register print function for debugging */
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
  lv_label_set_text(label, "COER");
  lv_obj_align(label, LV_ALIGN_CENTER, 0, 0);
//
lv_obj_t * img = lv_img_create(lv_scr_act());
lv_img_set_src(img, &Logo);
lv_obj_center(img); // or position manually

//
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
  lv_chart_set_point_count(chart, POINT_COUNT);                                            // Set the number of data points
  // Hide grid lines
lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_X, 0, 0, 0, 0, false, 0);
lv_chart_set_axis_tick(chart, LV_CHART_AXIS_PRIMARY_Y, 0, 0, 0, 0, false, 0);
lv_obj_set_style_line_opa(chart, LV_OPA_TRANSP, LV_PART_MAIN | LV_STATE_DEFAULT);
  // Remove the points by setting LV_PART_INDICATOR style radius to 0
lv_obj_set_style_radius(chart, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
lv_obj_set_style_pad_all(chart, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
lv_obj_set_style_width(chart, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
lv_obj_set_style_height(chart, 0, LV_PART_INDICATOR | LV_STATE_DEFAULT);
  // Add a series for the chart
  fdc_series = lv_chart_add_series(chart, lv_palette_main(LV_PALETTE_GREEN), LV_CHART_AXIS_PRIMARY_Y);

  // Fill with initial data (flat line)
    for (int i = 0; i < POINT_COUNT; i++) {
        fdc_series->y_points[i] = 0;
    }
    lv_chart_refresh(chart);

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

  // Initialize the QMI8658
  if (!qmi.begin(Wire, QMI8658_L_SLAVE_ADDRESS, IIC_SDA, IIC_SCL))
  {

    while (1)
    {
      delay(1000);
    }
  }

  qmi.configAccelerometer(
      SensorQMI8658::ACC_RANGE_4G,
      SensorQMI8658::ACC_ODR_1000Hz,
      SensorQMI8658::LPF_MODE_0);

  qmi.enableAccelerometer();
  qmi.dumpCtrlRegister();

  // Initialize BLE
  BLEDevice::init("COER_WEARABLE_DEVICE");

  // Create BLE server
  BLEServer *pServer = BLEDevice::createServer();

  pServer->setCallbacks(new MyServerCallbacks());

  // Create BLE service
  BLEService *pService = pServer->createService(SERVICE_UUID);

  // Create RX characteristic
  BLECharacteristic *pRxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_RX,
      BLECharacteristic::PROPERTY_WRITE);
  pRxCharacteristic->setCallbacks(new MyCallbacks());

  // Create TX characteristic
  pTxCharacteristic = pService->createCharacteristic(
      CHARACTERISTIC_UUID_TX,
      BLECharacteristic::PROPERTY_NOTIFY);

  pTxCharacteristic->addDescriptor(new BLE2902());

  pService->start();

  // Start advertising
  BLEAdvertising *pAdvertising = BLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(SERVICE_UUID);
  pAdvertising->start();

  // Make Queues
  sensorQueueFast = xQueueCreate(20, sizeof(String));
  sensorQueueSlow = xQueueCreate(10, sizeof(String));

  xTaskCreate(accelerometer_task, "Accelerometer Task", 8192, NULL, 1, NULL);
  xTaskCreate(fdc_task, "FDC Task", 4096, NULL, 1, NULL);
  xTaskCreate(gui_task, "GUI Task", 4096, NULL, 1, NULL);
  xTaskCreate(TaskSendData, "TaskSendData", 4096, NULL, 2, NULL);
}

void loop()
{
  // put your main code here, to run repeatedly:
}
