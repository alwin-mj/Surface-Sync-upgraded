/*
 * SurfaceSync – ESP32-WROOM-32D Receiver Firmware  v2.2
 * =======================================================
 * HARDWARE (from your breadboard):
 *   Piezo center (+)  → MAX9814 IN+
 *   Piezo outer  (-)  → MAX9814 IN-  AND  ESP32 GND
 *   MAX9814 VCC       → ESP32 3.3V
 *   MAX9814 GND       → ESP32 GND
 *   MAX9814 GAIN      → ESP32 GND   (50dB gain)
 *   MAX9814 OUT       → 1MΩ → junction row J
 *   1N4148 #1: black band→3.3V rail, other leg→row J
 *   1N4148 #2: black band→row J,     other leg→GND
 *   Row J             → ESP32 GPIO34
 *
 * WHY TIMER + analogRead INSTEAD OF adc_continuous:
 *   adc_continuous at 44100 Hz is unreliable on WROOM in core v3.x —
 *   frequently returns ESP_ERR_TIMEOUT and drops frames silently.
 *   Hardware timer ISR + analogRead is simple, reliable, proven.
 *
 * SAMPLE RATE: 20000 Hz (20 kHz)
 *   Nyquist = 10 kHz — covers all 8-FSK tones (max 9500 Hz).
 *   More reliable than 44100 Hz with timer+analogRead on Xtensa.
 *   receiver_backend.py SAMPLE_RATE must also be 20000.
 *
 * SERIAL FRAME:
 *   [0xBB][0x44][LEN_HI][LEN_LO][int16 BE...][CRC_HI][CRC_LO]
 *   LEN_HI bit7=1 when RMS > PREAMBLE_THRESHOLD
 *
 * Board: ESP32 Dev Module, Core: v3.x
 */

#include <Arduino.h>

#define ADC_PIN           34
#define SERIAL_BAUD       921600
#define SAMPLE_RATE       20000
#define FRAME_SAMPLES     256
#define ADC_MID           2048
#define PREAMBLE_THRESHOLD 150   // ADC counts RMS — ~7% full scale

#define FRAME_SYNC_0  0xBB
#define FRAME_SYNC_1  0x44
#define CALIB_DONE    0xCA
#define CMD_CALIBRATE 0xC2
#define CMD_RESET     0xC3

// Ring buffer
#define RING_SIZE  2048
static volatile int16_t  ring_buf[RING_SIZE];
static volatile uint16_t ring_wr = 0;
static          uint16_t ring_rd = 0;

static int16_t frame_buf[FRAME_SAMPLES];
static uint8_t serial_buf[FRAME_SAMPLES * 2 + 8];

static uint16_t crc16(const uint8_t* d, uint16_t n) {
    uint16_t c = 0xFFFF;
    for (uint16_t i = 0; i < n; i++) {
        c ^= d[i];
        for (uint8_t j = 0; j < 8; j++)
            c = (c & 1) ? ((c >> 1) ^ 0xA001) : (c >> 1);
    }
    return c;
}

static void send_frame(int16_t* s, uint16_t n, bool pre) {
    uint16_t bl = n * 2;
    for (uint16_t i = 0; i < n; i++) {
        serial_buf[i*2]   = (uint8_t)((s[i] >> 8) & 0xFF);
        serial_buf[i*2+1] = (uint8_t)(s[i] & 0xFF);
    }
    uint16_t crc = crc16(serial_buf, bl);
    uint16_t lw  = bl | (pre ? 0x8000 : 0);
    uint8_t  hdr[4]   = { FRAME_SYNC_0, FRAME_SYNC_1,
                          (uint8_t)(lw >> 8), (uint8_t)(lw & 0xFF) };
    uint8_t  trail[2] = { (uint8_t)(crc >> 8), (uint8_t)(crc & 0xFF) };
    Serial.write(hdr,        4);
    Serial.write(serial_buf, bl);
    Serial.write(trail,      2);
}

static hw_timer_t*  g_timer = NULL;
static portMUX_TYPE g_mux   = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&g_mux);
    int16_t s = (int16_t)(analogRead(ADC_PIN) - ADC_MID);
    ring_buf[ring_wr & (RING_SIZE - 1)] = s;
    ring_wr++;
    portEXIT_CRITICAL_ISR(&g_mux);
}

void setup() {
    Serial.begin(SERIAL_BAUD, SERIAL_8N1);
    analogReadResolution(12);
    analogSetAttenuation(ADC_6db);
    pinMode(ADC_PIN, INPUT);

    g_timer = timerBegin(SAMPLE_RATE);
    timerAttachInterrupt(g_timer, &onTimer);
    timerAlarm(g_timer, 1, true, 0);

    delay(300);
    Serial.write(CALIB_DONE);  // ready signal to Python
}

void loop() {
    static uint16_t fi = 0;

    while (true) {
        uint16_t wr;
        portENTER_CRITICAL(&g_mux);
        wr = ring_wr;
        portEXIT_CRITICAL(&g_mux);
        if ((uint16_t)(wr - ring_rd) == 0) break;

        frame_buf[fi++] = ring_buf[ring_rd & (RING_SIZE - 1)];
        ring_rd++;

        if (fi >= FRAME_SAMPLES) {
            int64_t sq = 0;
            for (int j = 0; j < FRAME_SAMPLES; j++)
                sq += (int32_t)frame_buf[j] * frame_buf[j];
            float rms = sqrtf((float)sq / FRAME_SAMPLES);
            send_frame(frame_buf, FRAME_SAMPLES, rms > PREAMBLE_THRESHOLD);
            fi = 0;
        }
    }

    while (Serial.available()) {
        switch (Serial.read()) {
            case CMD_CALIBRATE:
                delay(5100);
                Serial.write(CALIB_DONE);
                break;
            case CMD_RESET:
                delay(100); ESP.restart(); break;
        }
    }
}
