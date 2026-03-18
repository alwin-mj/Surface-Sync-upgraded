/*
 * SurfaceSync – ESP32-CAM Transmitter Firmware  v2.1  [8-FSK, GPIO12]
 * =====================================================================
 *
 * YOUR EXACT WIRING (from connection table):
 * ───────────────────────────────────────────────────────────────────
 *  FTDI TX       → ESP32-CAM U0R  (GPIO3)          [Yellow]
 *  FTDI RX       → ESP32-CAM U0T  (GPIO1)          [Green]
 *  FTDI GND      → ESP32-CAM GND                   [Black]
 *  FTDI 5V       → ESP32-CAM 5V                    [Red]
 *
 *  ESP32-CAM GPIO12 → PAM8403 Left IN+             [Orange]  ← AUDIO OUT
 *  ESP32-CAM GND    → PAM8403 Left IN-             [Black]
 *  ESP32-CAM 5V     → PAM8403 VCC                  [Red]
 *  ESP32-CAM GND    → PAM8403 GND                  [Black]
 *
 *  PAM8403 LOUT+    → Exciter +                    [Red]
 *  PAM8403 LOUT-    → Exciter -                    [Black]
 * ───────────────────────────────────────────────────────────────────
 *
 * WHY NOT I2S DAC?
 *   The ESP32 built-in I2S DAC is HARDWARE-LOCKED to GPIO25 (DAC1) and
 *   GPIO26 (DAC2). You cannot remap it. Since your audio wire goes to
 *   GPIO12, we use the LEDC (PWM) peripheral instead, which works on
 *   ANY GPIO including GPIO12.
 *
 *   Method: High-frequency PWM carrier (312.5 kHz, 8-bit) with duty
 *   cycle updated at SAMPLE_RATE (20 kHz) via a hardware timer ISR.
 *   A simple RC low-pass filter on GPIO12 (10kΩ + 10nF) smooths the
 *   PWM into a clean analogue sine wave before it hits the PAM8403.
 *   The PAM8403's own input capacitance already acts as a partial LPF,
 *   so you will get audio even without an RC filter – add one if you
 *   hear buzzing.
 *
 * SAMPLE RATE:
 *   20,000 Hz (reduced from 44100 to keep ISR load low on the Xtensa
 *   core while generating tones up to 9500 Hz – Nyquist OK at 20 kHz).
 *   Symbol rate stays 1000 baud → 20 samples per symbol.
 *
 * 8-FSK MODULATION:
 *   8 tones (3 bits/symbol) + 1 preamble tone.
 *   Frequencies sent by Python backend via CMD_SET_FREQ (9 × uint16 BE).
 *   Default (glass): 2000,2750,3500,4250,5000,5750,6500,7250,8000 Hz.
 *
 * SERIAL PROTOCOL (unchanged – matches Python transmitter_backend.py):
 *   Frame: [0xAA][0x55][CMD][LEN_HI][LEN_LO][DATA...][CRC_HI][CRC_LO]
 *   CMD_SWEEP    0x01
 *   CMD_SET_FREQ 0x02  (9 × uint16 BE = 18 bytes)
 *   CMD_PACKET   0x03
 *   CMD_RESET    0x04
 *
 * ARDUINO IDE SETUP (read the bottom of this file for step-by-step):
 *   Board   : AI Thinker ESP32-CAM
 *   CPU Freq: 240 MHz
 *   Port    : whichever COM port your FTDI appears as
 *   Upload  : pull GPIO0 LOW during upload, then release after reset
 *
 * Dependencies: ESP32 Arduino core >= 2.0 (built-in, no extra libs needed)
 */

#include <Arduino.h>

// ═══════════════════════════════════════════════════════════════════
//  PIN DEFINITIONS  – match your exact wiring
// ═══════════════════════════════════════════════════════════════════
#define AUDIO_PIN         12    // GPIO12 → PAM8403 Left IN+  (orange wire)
#define SERIAL_RX_PIN      3    // GPIO3  = U0R (FTDI TX connects here)
#define SERIAL_TX_PIN      1    // GPIO1  = U0T (FTDI RX connects here)

// ═══════════════════════════════════════════════════════════════════
//  LEDC (PWM DAC) CONFIG
// ═══════════════════════════════════════════════════════════════════
#define LEDC_CHANNEL      0
#define LEDC_TIMER_BITS   8                   // 8-bit resolution (0-255)
#define LEDC_BASE_FREQ    312500              // PWM carrier: 312.5 kHz
                                              // 80 MHz / 256 = 312500 Hz

// ═══════════════════════════════════════════════════════════════════
//  AUDIO SAMPLE RATE & 8-FSK CONSTANTS
// ═══════════════════════════════════════════════════════════════════
#define SAMPLE_RATE       20000    // Hz  – ISR fires at this rate
#define BAUD_RATE         1000     // symbols/s
#define SAMPLES_PER_SYM   (SAMPLE_RATE / BAUD_RATE)   // = 20 samples
#define N_SYMBOLS         8
#define FREQ_SPACING      750
#define PREAMBLE_MS       20
#define GUARD_MS          3
#define SILENCE_MS        5
#define DAC_MID           128     // PWM mid-point = 0 V AC
#define DAC_AMP           100     // Sine amplitude (0–127); 100 = ~78% of range

// ═══════════════════════════════════════════════════════════════════
//  SERIAL PROTOCOL BYTES
// ═══════════════════════════════════════════════════════════════════
#define SERIAL_BAUD       921600
#define SYNC_0            0xAA
#define SYNC_1            0x55
#define ACK_BYTE          0x06
#define NACK_BYTE         0x15
#define SWEEP_DONE        0xA0
#define CMD_SWEEP         0x01
#define CMD_SET_FREQ      0x02
#define CMD_PACKET        0x03
#define CMD_RESET         0x04

// ═══════════════════════════════════════════════════════════════════
//  8-FSK FREQUENCY TABLE  (default = glass surface)
// ═══════════════════════════════════════════════════════════════════
static uint16_t g_freqs[9] = {
    2000, 2750, 3500, 4250, 5000, 5750, 6500, 7250, 8000
    //  sym0  sym1  sym2  sym3  sym4  sym5  sym6  sym7  preamble
};

// ═══════════════════════════════════════════════════════════════════
//  SINE LOOK-UP TABLE  (256 entries, 8-bit unsigned, DC-biased)
//  Generated once at boot so the ISR never calls sinf().
// ═══════════════════════════════════════════════════════════════════
static uint8_t sine_lut[256];

static void build_sine_lut() {
    for (int i = 0; i < 256; i++) {
        float angle = 2.0f * 3.14159265f * i / 256.0f;
        sine_lut[i] = (uint8_t)(DAC_MID + (int)(DAC_AMP * sinf(angle)));
    }
}

// ═══════════════════════════════════════════════════════════════════
//  ISR-DRIVEN TONE PLAYBACK
//  A hardware timer fires every 1/SAMPLE_RATE seconds.
//  Each ISR tick advances the phase accumulator by phase_inc and
//  writes the corresponding LUT value to the LEDC duty register.
// ═══════════════════════════════════════════════════════════════════
static volatile uint32_t g_phase_acc  = 0;   // Q16.16 fixed-point accumulator
static volatile uint32_t g_phase_inc  = 0;   // phase step per ISR tick
static volatile uint32_t g_samples_left = 0; // countdown; 0 = idle
static volatile bool      g_tone_done   = false;

// Phase increment for a given frequency (Q16.16):
//   phase_inc = (freq * 256 * 65536) / SAMPLE_RATE
// The LUT index is the top 8 bits of the accumulator.
static inline uint32_t freq_to_inc(uint16_t freq_hz) {
    return (uint32_t)(((uint64_t)freq_hz * 256ULL * 65536ULL) / SAMPLE_RATE);
}

static hw_timer_t* g_timer = NULL;
static portMUX_TYPE g_timer_mux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR onTimer() {
    portENTER_CRITICAL_ISR(&g_timer_mux);
    if (g_samples_left > 0) {
        g_phase_acc += g_phase_inc;
        uint8_t lut_idx = (uint8_t)(g_phase_acc >> 16);   // top 8 bits
        ledcWrite(LEDC_CHANNEL, sine_lut[lut_idx]);
        g_samples_left--;
        if (g_samples_left == 0) {
            g_tone_done = true;
            ledcWrite(LEDC_CHANNEL, DAC_MID);   // silence immediately
        }
    }
    portEXIT_CRITICAL_ISR(&g_timer_mux);
}

// ── Start tone (non-blocking – returns immediately, ISR plays it) ──
static void start_tone(uint16_t freq_hz, uint32_t num_samples) {
    portENTER_CRITICAL(&g_timer_mux);
    g_phase_inc   = freq_to_inc(freq_hz);
    g_samples_left = num_samples;
    g_tone_done   = false;
    portEXIT_CRITICAL(&g_timer_mux);
}

// ── Block until tone finishes ──────────────────────────────────────
static void wait_tone_done() {
    while (!g_tone_done) { /* spin */ }
}

// ── Play tone synchronously (blocking) ────────────────────────────
static void play_tone(uint16_t freq_hz, uint32_t num_samples) {
    start_tone(freq_hz, num_samples);
    wait_tone_done();
}

// ── Play silence (blocking) ────────────────────────────────────────
static void play_silence(uint32_t num_samples) {
    ledcWrite(LEDC_CHANNEL, DAC_MID);
    // Use tone with 0 phase increment = DC = silence
    // Simpler: just delay the equivalent time
    uint32_t us = (uint32_t)(((uint64_t)num_samples * 1000000ULL) / SAMPLE_RATE);
    delayMicroseconds(us);
}

// ═══════════════════════════════════════════════════════════════════
//  TIMER INIT
// ═══════════════════════════════════════════════════════════════════
static void timer_init() {
    // Timer 0, prescaler 4 → tick at 80 MHz / 4 = 20 MHz
    // Alarm every 20 MHz / SAMPLE_RATE = 20000000 / 20000 = 1000 ticks
    g_timer = timerBegin(0, 4, true);
    timerAttachInterrupt(g_timer, &onTimer, true);
    timerAlarmWrite(g_timer, 20000000UL / SAMPLE_RATE, true);
    timerAlarmEnable(g_timer);
}

// ═══════════════════════════════════════════════════════════════════
//  CRC-16 / IBM
// ═══════════════════════════════════════════════════════════════════
static uint16_t crc16(const uint8_t* data, uint16_t len) {
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++)
            crc = (crc & 1) ? ((crc >> 1) ^ 0xA001) : (crc >> 1);
    }
    return crc;
}

// ═══════════════════════════════════════════════════════════════════
//  8-FSK PACKET TRANSMITTER
//  3 bytes → 8 symbols (3 bits each) → 8 tones played sequentially
// ═══════════════════════════════════════════════════════════════════
static void transmit_3bytes(uint8_t b0, uint8_t b1, uint8_t b2) {
    uint8_t syms[8];
    syms[0] = (b0 >> 5) & 0x07;
    syms[1] = (b0 >> 2) & 0x07;
    syms[2] = ((b0 & 0x03) << 1) | ((b1 >> 7) & 0x01);
    syms[3] = (b1 >> 4) & 0x07;
    syms[4] = (b1 >> 1) & 0x07;
    syms[5] = ((b1 & 0x01) << 2) | ((b2 >> 6) & 0x03);
    syms[6] = (b2 >> 3) & 0x07;
    syms[7] =  b2 & 0x07;
    for (int s = 0; s < 8; s++)
        play_tone(g_freqs[syms[s]], SAMPLES_PER_SYM);
}

static void transmit_packet(const uint8_t* data, uint16_t len) {
    // ── Preamble burst ───────────────────────────────────────────
    play_tone(g_freqs[8], (SAMPLE_RATE * PREAMBLE_MS) / 1000);
    play_silence((SAMPLE_RATE * GUARD_MS) / 1000);

    // ── Data: 3 bytes at a time → 8 symbols ─────────────────────
    uint16_t i = 0;
    while (i + 2 < len) {
        transmit_3bytes(data[i], data[i+1], data[i+2]);
        i += 3;
    }
    // ── Remaining bytes (0, 1, or 2): pad with 0x00 ─────────────
    if (i < len) {
        uint8_t r[3] = {0, 0, 0};
        uint8_t n = 0;
        while (i < len) r[n++] = data[i++];
        transmit_3bytes(r[0], r[1], r[2]);
    }

    // ── Post-packet silence ──────────────────────────────────────
    play_silence((SAMPLE_RATE * SILENCE_MS) / 1000);
}

// ═══════════════════════════════════════════════════════════════════
//  SURFACE SWEEP
// ═══════════════════════════════════════════════════════════════════
static void do_surface_sweep() {
    uint16_t sweep_freqs[] = {
        1000, 1500, 2000, 2500, 3000, 3500, 4000, 5000, 6000, 7000, 8000, 9000
    };
    uint32_t tone_samps = (SAMPLE_RATE * 200) / 1000;   // 200 ms per tone
    uint32_t gap_samps  = (SAMPLE_RATE * 20)  / 1000;   // 20 ms gap
    for (uint8_t i = 0; i < 12; i++) {
        play_tone(sweep_freqs[i], tone_samps);
        play_silence(gap_samps);
    }
    Serial.write(SWEEP_DONE);
}

// ═══════════════════════════════════════════════════════════════════
//  SERIAL FRAME PARSER
// ═══════════════════════════════════════════════════════════════════
#define MAX_FRAME_DATA   2048
static uint8_t  rx_buf[MAX_FRAME_DATA + 8];
static uint16_t rx_len = 0;

static void handle_frame(uint8_t cmd, const uint8_t* data, uint16_t len) {
    switch (cmd) {
        case CMD_SWEEP:
            Serial.write(ACK_BYTE);
            do_surface_sweep();
            break;

        case CMD_SET_FREQ:
            if (len == 18) {
                for (uint8_t i = 0; i < 9; i++)
                    g_freqs[i] = ((uint16_t)data[i*2] << 8) | data[i*2+1];
                Serial.write(ACK_BYTE);
            } else {
                Serial.write(NACK_BYTE);
            }
            break;

        case CMD_PACKET:
            Serial.write(ACK_BYTE);
            transmit_packet(data, len);
            break;

        case CMD_RESET:
            Serial.write(ACK_BYTE);
            delay(100);
            ESP.restart();
            break;

        default:
            Serial.write(NACK_BYTE);
            break;
    }
}

static void process_serial() {
    while (Serial.available()) {
        uint8_t b = Serial.read();
        if (rx_len < sizeof(rx_buf))
            rx_buf[rx_len++] = b;

        if (rx_len < 2) continue;

        // Sync word hunt
        if (rx_buf[0] != SYNC_0 || rx_buf[1] != SYNC_1) {
            uint16_t skip = 1;
            while (skip < rx_len && rx_buf[skip] != SYNC_0) skip++;
            memmove(rx_buf, rx_buf + skip, rx_len - skip);
            rx_len -= skip;
            continue;
        }

        if (rx_len < 5) continue;

        uint8_t  cmd      = rx_buf[2];
        uint16_t data_len = ((uint16_t)rx_buf[3] << 8) | rx_buf[4];
        uint16_t total    = 5 + data_len + 2;

        if (data_len > MAX_FRAME_DATA) { rx_len = 0; continue; }
        if (rx_len < total) continue;

        const uint8_t* payload  = rx_buf + 5;
        uint16_t       rx_crc   = ((uint16_t)rx_buf[5 + data_len] << 8)
                                 | rx_buf[5 + data_len + 1];

        if (crc16(payload, data_len) != rx_crc) {
            Serial.write(NACK_BYTE);
        } else {
            handle_frame(cmd, payload, data_len);
        }

        memmove(rx_buf, rx_buf + total, rx_len - total);
        rx_len -= total;
    }
}

// ═══════════════════════════════════════════════════════════════════
//  SETUP & LOOP
// ═══════════════════════════════════════════════════════════════════
void setup() {
    // Serial on GPIO1(TX) / GPIO3(RX) – matches your FTDI wiring
    Serial.begin(SERIAL_BAUD, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);

    // LEDC PWM on GPIO12 → PAM8403 Left IN+
    ledcSetup(LEDC_CHANNEL, LEDC_BASE_FREQ, LEDC_TIMER_BITS);
    ledcAttachPin(AUDIO_PIN, LEDC_CHANNEL);
    ledcWrite(LEDC_CHANNEL, DAC_MID);   // start silent

    // Build sine look-up table
    build_sine_lut();

    // Start ISR-driven sample timer
    timer_init();

    // ── Startup beep: 3 quick tones to confirm audio path is alive ──
    // You should hear 3 short beeps from the exciter on power-on.
    delay(200);
    play_tone(1000, SAMPLE_RATE / 10);   // 100 ms @ 1 kHz
    play_silence(SAMPLE_RATE / 20);
    play_tone(2000, SAMPLE_RATE / 10);   // 100 ms @ 2 kHz
    play_silence(SAMPLE_RATE / 20);
    play_tone(3000, SAMPLE_RATE / 10);   // 100 ms @ 3 kHz
    play_silence(SAMPLE_RATE / 20);
}

void loop() {
    process_serial();
    delay(1);
}

/*
 ╔══════════════════════════════════════════════════════════════════════╗
 ║          ARDUINO IDE STEP-BY-STEP UPLOAD GUIDE                      ║
 ╠══════════════════════════════════════════════════════════════════════╣
 ║                                                                      ║
 ║  STEP 1 – Install ESP32 board package (one-time)                     ║
 ║  ─────────────────────────────────────────────────────────────────   ║
 ║  Arduino IDE → File → Preferences                                    ║
 ║  Paste this URL in "Additional Boards Manager URLs":                 ║
 ║  https://raw.githubusercontent.com/espressif/arduino-esp32/         ║
 ║  gh-pages/package_esp32_index.json                                   ║
 ║  Then: Tools → Board → Boards Manager → search "esp32" → Install    ║
 ║                                                                      ║
 ║  STEP 2 – Select the correct board                                   ║
 ║  ─────────────────────────────────────────────────────────────────   ║
 ║  Tools → Board → ESP32 Arduino → AI Thinker ESP32-CAM               ║
 ║  Tools → CPU Frequency → 240 MHz                                     ║
 ║  Tools → Flash Mode → QIO                                            ║
 ║  Tools → Flash Size → 4MB (32Mb)                                     ║
 ║  Tools → Partition Scheme → Huge APP (3MB No OTA)                   ║
 ║                                                                      ║
 ║  STEP 3 – Select your COM port                                       ║
 ║  ─────────────────────────────────────────────────────────────────   ║
 ║  Tools → Port → (pick the port that appeared when you plugged        ║
 ║  the FTDI in. On Windows it looks like COM3, COM4, etc.              ║
 ║  On Linux/Mac it looks like /dev/ttyUSB0)                            ║
 ║                                                                      ║
 ║  STEP 4 – Enter bootloader mode BEFORE uploading                     ║
 ║  ─────────────────────────────────────────────────────────────────   ║
 ║  The ESP32-CAM has NO auto-reset circuit, so you must do this        ║
 ║  manually every time you flash:                                      ║
 ║                                                                      ║
 ║  a) Connect a wire/jumper from GPIO0 to GND                          ║
 ║     (GPIO0 is the boot mode pin – pulling it LOW = flash mode)       ║
 ║                                                                      ║
 ║  b) Press the RESET button on the ESP32-CAM board                    ║
 ║     (or briefly disconnect/reconnect the 5V power)                   ║
 ║                                                                      ║
 ║  c) Click Upload (→) in Arduino IDE                                  ║
 ║                                                                      ║
 ║  d) Watch the output. When you see "Connecting........___"           ║
 ║     the IDE is trying to talk to the chip. It should connect         ║
 ║     automatically.                                                   ║
 ║                                                                      ║
 ║  e) When upload is 100% done → REMOVE the GPIO0-GND jumper           ║
 ║     → Press RESET again. The new firmware boots.                     ║
 ║                                                                      ║
 ║  STEP 5 – Confirm it works                                           ║
 ║  ─────────────────────────────────────────────────────────────────   ║
 ║  After RESET (without GPIO0 grounded) you should hear THREE BEEPS    ║
 ║  from the exciter/speaker: low (1kHz) → mid (2kHz) → high (3kHz).   ║
 ║  This confirms GPIO12 → PAM8403 → Exciter chain is working.         ║
 ║                                                                      ║
 ║  If you hear nothing:                                                ║
 ║    • Check GPIO12 wire is in PAM8403 Left IN+ (not GND)             ║
 ║    • Check PAM8403 VCC is getting 5V                                 ║
 ║    • Check LOUT+ → Exciter+ and LOUT- → Exciter-                    ║
 ║    • Open Arduino Serial Monitor at 921600 baud → should be silent   ║
 ║      (no garbage = good, it means serial is working)                 ║
 ║                                                                      ║
 ║  STEP 6 – Run the Python backend                                     ║
 ║  ─────────────────────────────────────────────────────────────────   ║
 ║  python transmitter_backend.py                                        ║
 ║  Open transmitter.html in browser                                    ║
 ║  Connect serial in the UI (pick the same COM port)                   ║
 ║  Drop a .txt file and click Send                                     ║
 ║                                                                      ║
 ╚══════════════════════════════════════════════════════════════════════╝
 */
