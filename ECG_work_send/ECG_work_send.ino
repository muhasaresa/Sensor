//////////////////////////////////////////////////////////////////////////////////////////
//
//    MAX30001 ESP-NOW ECG Sender - Sending RAW Values & Synced
//    Increased Sampling/Sending Rate
//
//    - Reads raw ECG data from MAX30001.
//    - Sends this raw signed long ECG value (as a string) and synchronized
//      timestamp (E<RAW_VALUE>T<timestamp>\n) via ESP-NOW.
//    - Receives sync packets from a master for timestamp synchronization.
//
/////////////////////////////////////////////////////////////////////////////////////////

#include <SPI.h>
#include "protocentral_max30001.h"

#include <esp_now.h>
#include <WiFi.h>
#include <esp_wifi.h>

// ESP32 Pin Configuration
#define MAX30001_MOSI_PIN 23
#define MAX30001_MISO_PIN 19
#define MAX30001_SCLK_PIN 18
#define MAX30001_CS_PIN   5
#define MAX30001_INT1_PIN 2

// Interval for reading from MAX30001 and sending ECG data via ESP-NOW
#define DATA_SEND_INTERVAL_MS 10 // Approx 100 Hz (was 100ms for 10Hz)
                                 // Adjust this based on desired detail vs. ESP-NOW load

MAX30001 max30001(MAX30001_CS_PIN);

signed long ecg_data_raw_current;
bool sensorInitialized = false;

// ESP-NOW and Sync Variables
typedef struct {
    uint32_t sync_time;
} sync_packet_t;
uint8_t masterReceiverMacAddress[] = {0x80, 0x7D, 0x3A, 0xF3, 0xBF, 0x4C}; // Your Receiver's MAC
esp_now_peer_info_t masterPeerInfo;
const uint8_t WIFI_CHANNEL = 6;

volatile uint32_t current_synced_timestamp_ms = 0;
volatile bool time_synced_at_least_once = false;
unsigned long last_loop_run_millis_ref = 0;

void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    // if (status != ESP_NOW_SEND_SUCCESS) { Serial.printf("ECG ESP-NOW Send Failure: %d\n", status); }
}

void onSyncDataRecv(const esp_now_recv_info *recv_info, const uint8_t *incomingData, int len) {
    if (len == sizeof(sync_packet_t)) {
        sync_packet_t syncData;
        memcpy(&syncData, incomingData, sizeof(syncData));
        if (syncData.sync_time == 0) {
            current_synced_timestamp_ms = 0;
            last_loop_run_millis_ref = millis();
            time_synced_at_least_once = false;
            Serial.println("ECG: RESET sync received.");
        } else {
            current_synced_timestamp_ms = syncData.sync_time;
            last_loop_run_millis_ref = millis();
            time_synced_at_least_once = true;
            // Serial.printf("ECG: SYNC received. Master time: %lu\n", syncData.sync_time);
        }
    }
}

void setup() {
    Serial.begin(115200);
    unsigned long setup_start_time = millis();
    while (!Serial && (millis() - setup_start_time < 3000)) { delay(10); }
    Serial.println("\n--- MAX30001 ESP-NOW ECG Sender (RAW Values, Synced, 50Hz Target) ---");

    pinMode(MAX30001_CS_PIN, OUTPUT);
    digitalWrite(MAX30001_CS_PIN, HIGH);
    SPI.begin(MAX30001_SCLK_PIN, MAX30001_MISO_PIN, MAX30001_MOSI_PIN);
    Serial.println("SPI Initialized.");

    Serial.println("Detecting MAX30001...");
    if (max30001.max30001ReadInfo()) {
        Serial.println("MAX30001 Detected. Initializing...");
        // Consider if max30001.BeginECGBioZ(); needs different parameters for higher sampling rates
        // if the library supports it and if the default isn't already high enough.
        // For now, using default configuration.
        max30001.BeginECGBioZ();
        sensorInitialized = true;
        Serial.println("MAX30001 Initialized.");
    } else {
        Serial.println("FAILURE: MAX30001 not detected. Halting.");
        while (1) { delay(1000); }
    }

    WiFi.mode(WIFI_STA);
    delay(200); // Delay for MAC address stabilization
    Serial.print("ECG MAC: "); Serial.println(WiFi.macAddress());

    esp_err_t set_channel_err = esp_wifi_set_channel(WIFI_CHANNEL, WIFI_SECOND_CHAN_NONE);
    if (set_channel_err == ESP_OK) {
        Serial.printf("WiFi channel set to: %d\n", WIFI_CHANNEL);
    } else {
        Serial.print("Error setting WiFi channel to "); Serial.print(WIFI_CHANNEL);
        Serial.print(". Error: "); Serial.println(esp_err_to_name(set_channel_err));
        uint8_t actual_ch;
        esp_wifi_get_channel(&actual_ch, NULL);
        Serial.print("Device is currently on channel: "); Serial.println(actual_ch);
    }

    if (esp_now_init() != ESP_OK) {
        Serial.println("Error initializing ESP-NOW. Halting.");
        while (1) { delay(1000); }
    }
    Serial.println("ESP-NOW Initialized.");

    esp_now_register_send_cb(onDataSent);
    esp_now_register_recv_cb(onSyncDataRecv);

    memcpy(masterPeerInfo.peer_addr, masterReceiverMacAddress, 6);
    masterPeerInfo.channel = WIFI_CHANNEL; 
    masterPeerInfo.encrypt = false;
    if (esp_now_add_peer(&masterPeerInfo) != ESP_OK) {
        Serial.println("Failed to add master peer. Halting.");
        while (1) { delay(1000); }
    }
    Serial.println("Master peer added.");
    
    last_loop_run_millis_ref = millis();

    Serial.println("--- End of Setup ---");
    Serial.println("Starting RAW ECG data transmission at ~200Hz...");
}

void loop() {
    if (!sensorInitialized) {
        delay(1000); // Wait if sensor not ready
        return;
    }

    unsigned long current_millis_val = millis();

    // Update synced timestamp based on local millis progression
    if (time_synced_at_least_once) {
         current_synced_timestamp_ms += (current_millis_val - last_loop_run_millis_ref);
    }
    last_loop_run_millis_ref = current_millis_val;

    static unsigned long last_send_time_millis = 0; 
    if (current_millis_val - last_send_time_millis >= DATA_SEND_INTERVAL_MS) {
        last_send_time_millis = current_millis_val;

        if (time_synced_at_least_once || current_synced_timestamp_ms != 0) { // Send if time seems valid
            ecg_data_raw_current = max30001.getECGSamples(); // Get raw ECG data

            char data_buffer[50]; // Sufficient for "E<signed_long>T<uint32_t>\n"
            
            // Format the raw signed long ECG value and the timestamp
            snprintf(data_buffer, sizeof(data_buffer), "E%ldT%08lu\n", ecg_data_raw_current, current_synced_timestamp_ms);

            esp_now_send(masterReceiverMacAddress, (uint8_t *)data_buffer, strlen(data_buffer));
            
            // Optional Debugging:
            // Serial.printf("Sent RAW: %s", data_buffer); 
        }
    }
    // The main control for sampling rate is now the DATA_SEND_INTERVAL_MS check.
    // If DATA_SEND_INTERVAL_MS is very small (e.g. < 5ms), a tiny `delay(1)` might be 
    // beneficial to allow other tasks (like WiFi handling) to run, but for 20ms, it's likely fine without.
}