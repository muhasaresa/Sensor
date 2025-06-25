/*
 *  watch_diff_100sps_sync.ino
 *  ---------------------------------------------------------------
 *  • FDC1004 continuous differential (CH0 – CH3) @ 100 Hz
 *  • Every 10 ms read latest value, send "C±ddddddT########\n"
 *  • 4-byte sync packets from master keep time aligned
 *  • Wi-Fi fixed on channel 6  (matches ReceiverSync.ino)
 *  • Builds with ESP32-Arduino ≥ 3.2.0  on ESP32-S3
 */

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include <esp_now.h>
#include <Protocentral_FDC1004.h>
#include "pin_config.h"

/* ──────────  master MAC address  ────────── */
static constexpr uint8_t MAC_RX[6] = { 0x80,0x7D,0x3A,0xF3,0xBF,0x4C };

/* ──────────  FDC1004 registers  ────────── */
constexpr uint8_t MEAS_CONF0 = 0x08;
constexpr uint8_t MEAS0_MSB  = 0x00;

/* ──────────  time-sync state  ────────── */
static volatile uint32_t master_epoch_ms  = 0;
static volatile uint32_t epoch_ref_millis = 0;
static volatile bool     clock_locked     = false;

/* ──────────  FreeRTOS queues  ────────── */
struct SyncMsg { uint32_t ts; };
static QueueHandle_t syncQ;          // 8 entries
static QueueHandle_t txQ;            // 32 entries

/* ──────────  I²C helpers  ────────── */
inline void write16(uint8_t reg, uint16_t val)
{
    Wire.beginTransmission(0x50);
    Wire.write(reg);
    Wire.write(uint8_t(val >> 8));
    Wire.write(uint8_t(val));
    Wire.endTransmission();
}
inline uint16_t read16(uint8_t reg)
{
    Wire.beginTransmission(0x50); Wire.write(reg); Wire.endTransmission();
    Wire.requestFrom(0x50, uint8_t(2));
    return (uint16_t(Wire.read()) << 8) | Wire.read();
}

/* ──────────  ESP-NOW receive ISR  ────────── */
void onRecv(const esp_now_recv_info_t*, const uint8_t* data, int len)
{
    if (len != 4) return;
    SyncMsg m;  memcpy(&m.ts, data, 4);
    xQueueSendFromISR(syncQ, &m, nullptr);
}

/* ──────────  100 Hz sampler  ────────── */
void sampleTask(void*)
{
    const TickType_t period = pdMS_TO_TICKS(10);     // 10 ms = 100 Hz
    TickType_t next = xTaskGetTickCount();
    char line[32];

    for (;;)
    {
        int16_t raw = int16_t(read16(MEAS0_MSB));
        int32_t fF  = int32_t(raw) * 457 / 1000;     // 457 aF/LSB → fF

        uint32_t ts = clock_locked
                      ? master_epoch_ms + (millis() - epoch_ref_millis)
                      : 0;

        snprintf(line, sizeof(line),
                 "C%+06ldT%08lu\n", long(fF),
                 static_cast<unsigned long>(ts));

        xQueueSend(txQ, line, 0);                   // non-blocking
        vTaskDelayUntil(&next, period);
    }
}

/* ──────────  sender  ────────── */
void sendTask(void*)
{
    char line[32];
    for (;;)
        if (xQueueReceive(txQ, &line, portMAX_DELAY) == pdPASS)
            esp_now_send(MAC_RX,
                         reinterpret_cast<uint8_t*>(line),
                         strlen(line));
}

/* ──────────  sync handler  ────────── */
void syncTask(void*)
{
    SyncMsg m;
    for (;;)
    {
        xQueueReceive(syncQ, &m, portMAX_DELAY);
        if (m.ts == 0)
        {
            clock_locked = false;
            master_epoch_ms  = 0;
            epoch_ref_millis = millis();
        }
        else
        {
            master_epoch_ms  = m.ts;
            epoch_ref_millis = millis();
            clock_locked     = true;
        }
    }
}

/* ──────────  radio + ESP-NOW  ────────── */
void radioInit()
{
    WiFi.mode(WIFI_STA);
    esp_wifi_set_channel(6, WIFI_SECOND_CHAN_NONE);     // channel 6
    WiFi.disconnect(true, true);
}
void espNowInit()
{
    if (esp_now_init() != ESP_OK) { Serial.println("ESP-NOW init fail"); ESP.restart(); }

    esp_now_peer_info_t peer{};
    memcpy(peer.peer_addr, MAC_RX, 6);
    peer.channel = 6;  peer.encrypt = false;
    esp_now_add_peer(&peer);

    esp_now_register_recv_cb(onRecv);
}

/* ──────────  FDC1004 continuous diff @ 100 Hz  ────────── */
void fdcInit()
{
    Wire.begin(IIC_SDA, IIC_SCL, 400000UL);

    /* slot-0: CH0(+) – CH3(–) */
    write16(MEAS_CONF0, (0b000 << 13) | (0b011 << 10));

    /* continuous @ 100 Hz, REPEAT = 1, MEAS1 = slot-0 */
    uint16_t start = (FDC1004_100HZ << 10) | (1 << 8) | (1 << 7);
    write16(FDC_REGISTER, start);                     // macro from library
}

/* ──────────  Arduino life-cycle  ────────── */
void setup()
{
    Serial.begin(115200);
    radioInit();
    espNowInit();
    fdcInit();

    syncQ = xQueueCreate(8,  sizeof(SyncMsg));
    txQ   = xQueueCreate(32, sizeof(char[32]));

    xTaskCreatePinnedToCore(sampleTask, "sample", 2048, nullptr, 3, nullptr, 1);
    xTaskCreatePinnedToCore(sendTask,   "send",   3072, nullptr, 2, nullptr, 0);
    xTaskCreatePinnedToCore(syncTask,   "sync",   2048, nullptr, 2, nullptr, 1);

    Serial.println("Watch: 100 Hz streaming, waiting for sync.");
}

void loop() { vTaskDelay(pdMS_TO_TICKS(1000)); }
