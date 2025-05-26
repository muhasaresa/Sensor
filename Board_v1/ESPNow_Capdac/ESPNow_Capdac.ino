#include <Arduino.h>
#include <Wire.h>                         // I2C library
#include <Protocentral_FDC1004.h>         // FDC library
#include <Adafruit_Sensor.h>              // ADXL Library
#include <Adafruit_ADXL345_U.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h> // Include this for esp_wifi_set_max_tx_power
// Comment the following lines to disable the corresponding sensors
//#define ACCELEROMETER
//#define HEARTRATESENSOR
//#define GSRSENSOR
#define PROTOCENTRALFDC
#define ECGSENSOR


// #defines for FDC
#define UPPER_BOUND  0X4000                 // max readout capacitance
#define LOWER_BOUND  (-1 * UPPER_BOUND)
#define CHANNEL 3                          // channel to be read
#define MEASURMENT 0                       // measurment channel

// MAC address of the receiver (replace with your receiver’s MAC address)
uint8_t receiverAddress[] = {0x30, 0xAE, 0xA4, 0xE8, 0xAF, 0x5C};  // Example MAC; update it for your receiver
// FreeRTOS queue to pass data between tasks
QueueHandle_t sensorDataQueue;

// Task handles
TaskHandle_t Task1Handle = NULL;
TaskHandle_t Task2Handle = NULL;
TaskHandle_t nowTaskHandle = NULL;




//  Variables
const int PulseSensorPurplePin = 36;        // Pulse Sensor PURPLE WIRE connected to ANALOG PIN 0
const int GsrSensorPin = 39;
const float GsrCalibration = 1835.0;
const int EcgSensorL1 = 2;                  // ECG Sensor L1
const int EcgSensorL2 = 15;                  // ECG Sensor L2
const int EcgSensorPin = 34;                  // ECG Sensor Pin



int GsrSensorValue = 0;
int gsr_average = 0;

int capdac = 0;
char result[100];
const int maxCapDAC = 31;       // Maximum CAPDAC value
const float targetBaseline = 0; // Target baseline capacitance
const float tolerance = 1.0;    // Tolerance range around baseline
const int numSamples = 1000;    // Number of samples for averaging


unsigned long previousMillis = 0; 

#ifdef ACCELEROMETER
/* Assign a unique ID to this sensor at the same time */
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
#endif

// See the following for generating UUIDs:
// https://www.uuidgenerator.net/


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
  while (1) {

    // Serial.print("ts: " + String(ts));
    TickType_t startTick = xTaskGetTickCount();
    //Serial.print(String(ts));
    //Serial.print(", ");
    #ifdef HEARTRATESENSOR
    int Signal = analogRead(PulseSensorPurplePin);  // Read the PulseSensor's value.
    //Serial.print("P");
    //Serial.println(String(Signal));
    offset += snprintf(buffer+offset, sizeof(buffer), "P%04d",Signal);
    #endif
    
    #ifdef ECGSENSOR
    if((digitalRead(EcgSensorL1) == 1)||(digitalRead(EcgSensorL2) == 1)){
      Serial.println('!');
    }
    else{
      // send the value of analog input 0:

      int ecgSignal = analogRead(EcgSensorPin);
      vTaskDelay(pdMS_TO_TICKS(7)); // Wait for 7 ms
      offset += snprintf(buffer+offset, sizeof(buffer), "E%04d",ecgSignal);
      //Serial.println('ecgSignal');
    }
    //delay(20);
    //Serial.print(", ");
    #endif
    #ifdef ACCELEROMETER
      sensors_event_t event; 
      accel.getEvent(&event);
      offset  += snprintf(buffer+offset,sizeof(buffer), "X%+02.2fY%+02.2f%Z%+02.2f",event.acceleration.x,event.acceleration.y, event.acceleration.z);
      #endif
    #ifdef PROTOCENTRALFDC
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
      offset += snprintf(buffer+offset,sizeof(buffer),"C%05d",capacitance);
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
    #endif

    
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

void Task2(void *pvParameters) {
  uint32_t ulNotificationValue;

  while(1){
  
    // Wait for a notification on index 0 (Timer Interrupt)
    ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    // Get the start tick count
    // if((GlobalNotifyByte & 1) == 1){  
      TickType_t startTick = xTaskGetTickCount();

      #ifdef GSRSENSOR
      long sum = 0;
      float HumanResistance = 0;
      for(int i=0;i<10;i++) //Average the 10 measurements to remove the glitch
      {
        GsrSensorValue=analogRead(GsrSensorPin);
        sum += GsrSensorValue;
        delay(5);
      }
      gsr_average = sum/10;
      HumanResistance = ((4096 + 2*gsr_average) * 10000) / (GsrCalibration - gsr_average);
      #endif

      vTaskDelay(pdMS_TO_TICKS(20));         // Wait for 20 ms
      //Serial.print("[");
      #ifdef GSRSENSOR
      Serial.println(HumanResistance);
      //Serial.print(", ");
      #endif
      
      //Serial.println(" ]");

      TickType_t endTick = xTaskGetTickCount();

      // Calculate the elapsed ticks
      TickType_t elapsedTicks = endTick - startTick;

      // Convert ticks to milliseconds
      uint32_t elapsedTimeMs = elapsedTicks * (1000 / configTICK_RATE_HZ);
      //Serial.println("Time taken: " + String(elapsedTimeMs) );
    // }
  }
}

void nowTask(void * pvParameters) {
  char txbuffer[200];

  while (true) {
        // Wait until both Task 1 and Task 2 have finished sampling
        //Serial.print("Buffer");
        
      // Wait to receive sensor data from the analogReadTask
        if (xQueueReceive(sensorDataQueue, &txbuffer, portMAX_DELAY)) {
            // Set the BLE characteristic value with the received packet
            // Send data over ESP-NOW
            esp_err_t result = esp_now_send(receiverAddress, (uint8_t *)txbuffer, sizeof(txbuffer));
            Serial.print(txbuffer);
                    // Wait a short period to avoid flooding the receiver
            memset(txbuffer, 0, sizeof(txbuffer));
            //vTaskDelay(50 / portTICK_PERIOD_MS);
                    // Clear the buffer after sending
        


        }
    }
}



void setup() {

  // Initialize serial communication
  Serial.begin(115200);
      // Initialize WiFi in station mode
  WiFi.mode(WIFI_STA);
 
  WiFi.disconnect(); // Disconnect from any network (not needed for ESP-NOW)

    // Initialize the Wi-Fi driver
    if (esp_wifi_set_mode(WIFI_MODE_STA) != ESP_OK) {
        Serial.println("Error setting Wi-Fi mode");
        return;
    }
      // Set the maximum TX power (values: 8 - 84, in 0.25 dBm units)
    //esp_wifi_set_max_tx_power(50); // Example: 20 corresponds to 5 dBm
  Wire.begin();
  #ifdef ECGSENSOR
  pinMode(EcgSensorL1, INPUT); // Setup for leads off detection LO +
  pinMode(EcgSensorL2, INPUT); // Setup for leads off detection LO -
  #endif

  #ifdef ACCELEROMETER
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // accel.setRange(ADXL345_RANGE_8_G);
  // accel.setRange(ADXL345_RANGE_4_G);
  // accel.setRange(ADXL345_RANGE_2_G);
  #endif
      // Initialize ESP-NOW
    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW");
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
        Serial.println("Failed to add peer");
        return;
    }
    else {
    Serial.println("Peer added successfully");
  }

     // Create the sensor data queue
    sensorDataQueue = xQueueCreate(10, sizeof(char) * 200);  // Queue to store 10 data packets

        // Check if the queue was created successfully
    if (sensorDataQueue == NULL) {
        Serial.println("Error creating the queue.");
        while (1);  // Halt the program
    }

    // Create Task 1 (Highest priority)
    xTaskCreate(Task1, "Task1", 6000, NULL, 2, &Task1Handle);

    // Create Task 2 (Lower priority) Enable if needed
    //xTaskCreatePinnedToCore(Task2, "Task2", 1000, NULL, 2, &Task2Handle, 1);

    // Create Task 3 (Lowest priority)
    xTaskCreate(nowTask, "Task3", 6000, NULL, 1, &nowTaskHandle);


    
}

void loop() {
  // Nothing to do in the loop
}