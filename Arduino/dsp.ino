/*
 * audio_dsp.ino — ESP32-S3 + TLV320ADC6120 + 2x TAS5827
 *                 (dual standard-I2S, 4-channel DSP)
 * ==========================================================================
 *
 *  Signal chain (per audio frame, 48 kHz fixed):
 *    TLV320ADC6120  (standard stereo I2S, 32-bit, 24-in-32)
 *      -> 18-band shared parametric EQ (both channels)
 *      -> Routing: each of 4 outputs picks IN L, IN R, or silence
 *      -> Per-output gain (0..1)
 *      -> Per-output 8-band PEQ
 *      -> Per-output delay (0..25 ms @ 48 kHz = 0..1200 samples)
 *      -> Per-output phase invert
 *      -> Two standard stereo I2S streams:
 *           I2S0 TX (master) -> AMP1 SDIN = OUT 1 (L) + OUT 2 (R)
 *           I2S1 TX (slave)  -> AMP2 SDIN = OUT 3 (L) + OUT 4 (R)
 *
 *  Clocking (this is the change from the old TDM design):
 *    I2S0 is the I2S master and drives the shared BCLK + WS. The ADC (RX) and
 *    AMP1 (TX) live on I2S0. I2S1 is a TX-only SLAVE that reads the same
 *    BCLK/WS pins as inputs and clocks AMP2 — so both amps stay sample-locked.
 *    Standard stereo framing (the WS level identifies the channel) has no TDM
 *    slot-phase to latch wrong, which is what removes the boot-to-boot
 *    misalignment we had on the 4-slot TDM bus.
 *
 *  Hardware (ESP32-S3 Arduino core 3.x, IDF 5.x underneath):
 *    I2C:          SDA=GPIO8  SCL=GPIO9  @ 400 kHz
 *      ADC  TLV320ADC6120  = 0x4E
 *      AMP1 TAS5827        = 0x61
 *      AMP2 TAS5827        = 0x60
 *    I2S0 (master, full-duplex, standard stereo, 32-bit):
 *      BCLK   = GPIO4   (shared, driven by I2S0)
 *      WS     = GPIO5   (shared, driven by I2S0)
 *      DOUT   = GPIO6   -> AMP1 SDIN
 *      DIN    = GPIO7   <- ADC SDOUT
 *    I2S1 (slave, TX only, standard stereo, 32-bit):
 *      BCLK/WS = GPIO4/GPIO5 (shared, read as inputs)
 *      DOUT    = GPIO21 -> AMP2 SDIN
 *    AMP shared PDN: GPIO10 (board-inverted: LOW=on, HIGH=off)
 *
 *  Controls:
 *    Rotary encoder: A=GPIO15  B=GPIO16
 *    Button:         GPIO17 (active-low, internal pull-up)
 *      Press -> toggle volume-adjust mode
 *
 *  Status LED (WS2812 on GPIO12):
 *    Volume mode OFF:
 *      Green  = ready
 *      Red    = error (5 s flash / fatal latch)
 *    Volume mode ON:
 *      Gradient green (0%) -> yellow (50%) -> red (100%)
 *      Flashing red at 100%
 *
 *  Filter types (matches old UI):
 *    0=Peak  1=LowShelf  2=HighShelf  3=HighCut(LPF)
 *    4=LowCut(HPF)  5=Notch  6=Bypass
 *  HPF/LPF steepness:
 *    6=1st-order  12=2nd-order BW  24=LR4  48=LR8
 *
 *  Configuration: over USB serial (WebSerial UI) -> serviceSerialConsole()
 *
 *  Libraries:
 *    ArduinoJson  Preferences
 *    IDF drivers: i2c_master, i2s_std  (used directly from Arduino core 3.x)
 *
 *  Board: ESP32S3 Dev Module, USB CDC On Boot: Enabled
 */

#include <Arduino.h>
#include <math.h>

#include <Preferences.h>
#include <ArduinoJson.h>
#include <nvs_flash.h>
#include <Adafruit_NeoPixel.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/i2s_std.h"
#include "esp_heap_caps.h"
#include "esp_rom_gpio.h"
#include "soc/gpio_sig_map.h"

/* ==================================================================
 *  Pins
 * ================================================================== */
#define PIN_LED_RED          13
#define PIN_LED_GREEN        14

#define PIN_STATUS_LED       12        /* WS2812 data line */
#define PIN_BUTTON           17        /* GPIO17 — press toggles volume-adjust mode */

/* Rotary encoder */
#define PIN_ENC_A            15        /* Encoder channel A */
#define PIN_ENC_B            16        /* Encoder channel B */

#define PIN_I2C_SDA          GPIO_NUM_8
#define PIN_I2C_SCL          GPIO_NUM_9

#define PIN_I2S_BCLK         GPIO_NUM_4
#define PIN_I2S_FSYNC        GPIO_NUM_5     /* WS / LRCLK */
#define PIN_I2S_DOUT         GPIO_NUM_6     /* I2S0 TX -> AMP1 */
#define PIN_I2S_DOUT2        GPIO_NUM_21    /* I2S1 TX -> AMP2 (was the GPIO-matrix mirror) */
#define PIN_I2S_DIN          GPIO_NUM_7     /* I2S0 RX <- ADC */

#define PIN_AMP_PDN          GPIO_NUM_10
#define AMP_PDN_ON           0
#define AMP_PDN_OFF          1

/* ==================================================================
 *  Audio format — standard stereo I2S, 32-bit slots, 24-in-32 data
 * ================================================================== */
#define SAMPLE_RATE_HZ      48000
#define STD_SLOTS           2          /* L + R per frame */
#define BUFFER_FRAMES       16         /* small block -> low I/O latency; raise if dropouts */
#define PCM_SHIFT           8          /* 24-bit sample sits in the top 24 of 32 */

/* ==================================================================
 *  I2C addresses
 * ================================================================== */
#define ADC_I2C_ADDR     0x4E
#define AMP1_I2C_ADDR    0x61
#define AMP2_I2C_ADDR    0x60
#define I2C_FREQ_HZ      400000

/* ==================================================================
 *  TLV320ADC6120 registers (Page 0)
 * ================================================================== */
#define ADC_REG_PAGE_SEL      0x00
#define ADC_REG_SLEEP_CFG     0x02
#define ADC_REG_ASI_CFG0      0x07
#define ADC_REG_ASI_CFG1      0x08
#define ADC_REG_ASI_CFG2      0x09
#define ADC_REG_ASI_CH1       0x0B
#define ADC_REG_ASI_CH2       0x0C
#define ADC_REG_MST_CFG0      0x13
#define ADC_REG_DSP_CFG0      0x6B
#define ADC_REG_IN_CH_EN      0x73
#define ADC_REG_ASI_OUT_CH_EN 0x74
#define ADC_REG_PWR_CFG       0x75
#define ADC_REG_DEV_STS0      0x76

/* ==================================================================
 *  TAS5827 registers
 * ================================================================== */
#define TAS_REG_PAGE_SEL      0x00
#define TAS_REG_RESET         0x01
#define TAS_REG_DEVICE_CTRL1  0x02
#define TAS_REG_DEVICE_CTRL2  0x03
#define TAS_REG_SAP_CTRL1     0x33
#define TAS_REG_SAP_CTRL2     0x34
#define TAS_REG_SAP_CTRL3     0x35
#define TAS_REG_FS_MON        0x37
#define TAS_REG_BCK_MON       0x38
#define TAS_REG_CLKDET_STS    0x39
#define TAS_REG_DIG_VOL       0x4C
#define TAS_REG_AGAIN         0x54
#define TAS_REG_POWER_STATE   0x68
#define TAS_REG_CHAN_FAULT    0x70
#define TAS_REG_GLOBAL_FAULT1 0x71
#define TAS_REG_GLOBAL_FAULT2 0x72
#define TAS_REG_BOOK_SEL      0x7F

#define DCTRL2_MODE_HIZ       0x02
#define DCTRL2_MODE_PLAY      0x03

/* ==================================================================
 *  Persistent settings (NVS via Preferences)
 * ================================================================== */
static Preferences    prefs;

/* ==================================================================
 *  Status LED — single WS2812 on PIN_STATUS_LED
 * ================================================================== */
#define RGB(r, g, b)  ((uint32_t)(((uint8_t)(r) << 16) | \
                                  ((uint8_t)(g) << 8)  | \
                                   (uint8_t)(b)))

static const uint32_t STATUS_COLOR_ERROR    = RGB(255,   0,   0);
static const uint32_t STATUS_COLOR_READY    = RGB(  0, 255,  20);
static const uint32_t STATUS_COLOR_BLACK    = RGB(  0,   0,   0);

static const uint32_t ERROR_FLASH_MS        = 5000;

static Adafruit_NeoPixel statusLed(1, PIN_STATUS_LED, NEO_GRB + NEO_KHZ800);

static volatile uint32_t g_errorUntilMs = 0;
static volatile bool     g_errorFatal   = false;

static uint32_t          g_lastLedColorWritten = 0xFFFFFFFF;

/* ==================================================================
 *  Button + encoder state
 *
 *  Button (GPIO17, active-low, internal pull-up):
 *    Press (>= debounce) -> toggle volume-adjust mode
 *
 *  The button is sampled in loop() at ~10 ms intervals.
 *  Press detection:
 *    - Track when button first goes LOW (g_btnPressStartMs).
 *    - On release (LOW -> HIGH) decide short vs long by elapsed time.
 *    - Require button to return HIGH before next press registers.
 *
 *  Encoder (GPIO15=A, GPIO16=B):
 *    Read via interrupt on A. Direction determined by B state at A's
 *    falling edge (standard quadrature: B=LOW means CW/+, B=HIGH=CCW/-).
 *    A volatile step counter is accumulated; loop() drains it.
 * ================================================================== */

/* Button timing */
static const uint32_t BTN_LONG_PRESS_MS  = 3000;
static const uint32_t BTN_DEBOUNCE_MS    = 50;

static bool     g_btnWasLow        = false;   /* last sampled state */
static uint32_t g_btnPressStartMs  = 0;       /* millis() when button first went LOW */
static bool     g_btnArmed         = true;    /* false while held, reset on release */

/* Encoder — software quadrature debounce
 *
 *  A mechanical encoder bounces on both A and B contacts. A bare
 *  falling-edge ISR on A counts bounce spikes as real steps, so we
 *  use a full quadrature state machine instead:
 *
 *    State table (Gray code, reading A:B each ISR tick):
 *      00 -> 01 -> 11 -> 10 -> 00  =  clockwise (CW)
 *      00 -> 10 -> 11 -> 01 -> 00  =  counter-clockwise (CCW)
 *
 *  The ISR fires on ANY edge of A or B (CHANGE mode). At each
 *  transition we look up (prevState << 2 | curState) in a 16-entry
 *  table: +1 for a valid CW transition, -1 for CCW, 0 for invalid
 *  (bounce or missed step). We only accumulate the step counter when
 *  we complete a full detent cycle (4 valid transitions in the same
 *  direction), which rejects almost all mechanical bounce.
 *
 *  g_encSteps is written and read as a single aligned 32-bit word;
 *  on Xtensa that is atomic, so no mutex is needed between the ISR
 *  and loop(). */

/* Transition table: index = (prevAB << 2) | curAB, value = direction */
static const int8_t ENC_TABLE[16] DRAM_ATTR = {
/*       cur: 00  01  10  11          prev: */
/*  00 */     0, -1, +1,  0,
/*  01 */    +1,  0,  0, -1,
/*  10 */    -1,  0,  0, +1,
/*  11 */     0, +1, -1,  0,
};

static volatile int  g_encSteps    = 0;   /* +N = CW, -N = CCW; drained by loop() */
static volatile int8_t g_encState  = 0;   /* last AB reading (0..3) */
static volatile int  g_encAccum    = 0;   /* sub-detent accumulator */

/* Volume mode */
static bool     g_volumeMode       = false;   /* true while adjusting volume */
static uint32_t g_volumeModeExitMs = 0;       /* auto-exit after 5 s of no input */
static const uint32_t VOL_MODE_TIMEOUT_MS = 5000;

/* Volume step size: 5% of full scale (0..1) */
static const float VOL_STEP = 0.05f;

/* Flash state for 100% volume indicator */
static uint32_t g_volFlashNextMs   = 0;
static bool     g_volFlashOn       = true;
static const uint32_t VOL_FLASH_PERIOD_MS = 250;

/* ==================================================================
 *  IDF driver handles
 * ================================================================== */
static i2c_master_bus_handle_t s_i2c_bus  = NULL;
static i2c_master_dev_handle_t s_adc_dev  = NULL;
static i2c_master_dev_handle_t s_amp1_dev = NULL;
static i2c_master_dev_handle_t s_amp2_dev = NULL;

static i2s_chan_handle_t s_tx0_handle = NULL;   /* I2S0 TX -> AMP1 (OUT1/2) */
static i2s_chan_handle_t s_rx_handle  = NULL;   /* I2S0 RX <- ADC           */
static i2s_chan_handle_t s_tx1_handle = NULL;   /* I2S1 TX -> AMP2 (OUT3/4) */

/* ==================================================================
 *  DSP state
 * ================================================================== */
static SemaphoreHandle_t dspMutex = NULL;

volatile float volume = 1.0f;
/* Cached perceptual taper g_volGain = volume^2.5, recomputed only when the
 * volume changes, so the audio task never calls powf() (a flash function)
 * in its per-buffer path. */
volatile float g_volGain = 1.0f;

volatile float outputGain[4]  = {1.0f, 1.0f, 1.0f, 1.0f};
volatile bool  outputPhase[4] = {false, false, false, false};
volatile int   routing[4]     = {0, 1, 0, 1};

#define FTYPE_PEAK       0
#define FTYPE_LOW_SHELF  1
#define FTYPE_HIGH_SHELF 2
#define FTYPE_HIGH_CUT   3
#define FTYPE_LOW_CUT    4
#define FTYPE_NOTCH      5
#define FTYPE_BYPASS     6

struct Biquad {
    float b0, b1, b2, a1, a2;
    float s1, s2;
};

static inline void resetBiquad(Biquad &b) { b.s1 = b.s2 = 0.0f; }

static inline float IRAM_ATTR processBiquad(Biquad &b, float x) {
    float y  = b.b0 * x + b.s1;
    b.s1     = b.b1 * x - b.a1 * y + b.s2;
    b.s2     = b.b2 * x - b.a2 * y;
    return y;
}

#define NUM_EQ_BANDS 36

struct EQBand {
    volatile float freq;
    volatile float gain;
    volatile float q;
    volatile int   type;
    volatile int   steepness;
    volatile bool  active;
};

EQBand sharedEQ[NUM_EQ_BANDS];

static void initSharedEQ() {
    for (int i = 0; i < NUM_EQ_BANDS; i++) {
        sharedEQ[i] = {1000.0f, 0.0f, 0.7f, FTYPE_BYPASS, 12, false};
    }
}

#define NUM_OUTPUTS   4
#define MAX_OUT_BANDS 8

struct OutBand {
    volatile float freq;
    volatile float gain;
    volatile float q;
    volatile int   type;
    volatile int   steepness;
    volatile bool  active;
    volatile bool  on;
};

OutBand     outBands[NUM_OUTPUTS][MAX_OUT_BANDS];
volatile int outBandCount[NUM_OUTPUTS] = {0};

static void initOutBands() {
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        outBandCount[o] = 0;
        for (int i = 0; i < MAX_OUT_BANDS; i++) {
            outBands[o][i] = {1000.0f, 0.0f, 0.7f, FTYPE_BYPASS, 12, false, false};
        }
    }
}

#define DELAY_MAX_MS    25.0f
#define DELAY_MAX_SAMP  1201

volatile int delayInSamples[NUM_OUTPUTS] = {0};
float        delayBuf[NUM_OUTPUTS][DELAY_MAX_SAMP];
int          delayHead[NUM_OUTPUTS] = {0};

static void initDelayBuffers() {
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        delayInSamples[o] = 0;
        delayHead[o]      = 0;
        memset(delayBuf[o], 0, sizeof(delayBuf[o]));
    }
}

static inline float IRAM_ATTR processDelay(int o, float x) {
    int d = delayInSamples[o];
    if (d == 0) return x;
    delayBuf[o][delayHead[o]] = x;
    if (++delayHead[o] >= DELAY_MAX_SAMP) delayHead[o] = 0;
    int ri = delayHead[o] - d;
    if (ri < 0) ri += DELAY_MAX_SAMP;
    return delayBuf[o][ri];
}

DRAM_ATTR Biquad bqSharedL[NUM_EQ_BANDS][4];
DRAM_ATTR Biquad bqSharedR[NUM_EQ_BANDS][4];
DRAM_ATTR Biquad bqOut[NUM_OUTPUTS][MAX_OUT_BANDS][4];
DRAM_ATTR int    sharedEQStages[NUM_EQ_BANDS];
DRAM_ATTR int    outBandStages[NUM_OUTPUTS][MAX_OUT_BANDS];

DRAM_ATTR int activeSharedBands[NUM_EQ_BANDS];
DRAM_ATTR int activeSharedCount = 0;
DRAM_ATTR int activeOutBands[NUM_OUTPUTS][MAX_OUT_BANDS];
DRAM_ATTR int activeOutCount[NUM_OUTPUTS] = {0};

/* ==================================================================
 *  Level meter state
 * ================================================================== */
#define METER_DECAY        0.92f
#define METER_CLIP_THRESH  1.0f
#define METER_CLIP_HOLD_MS 1000

DRAM_ATTR static volatile float    g_meterPeak[NUM_OUTPUTS]        = {0};
DRAM_ATTR static volatile uint32_t g_meterClipUntilMs[NUM_OUTPUTS] = {0};

/* ==================================================================
 *  Click-suppression fade state
 * ================================================================== */
#define FADE_SAMPLES   240

enum FadeState : uint8_t {
    FADE_NONE = 0,
    FADE_DOWN,
    FADE_UP,
};

DRAM_ATTR static volatile uint32_t g_fadeRequest = 0;
DRAM_ATTR static uint32_t          g_fadeSeen    = 0;
DRAM_ATTR static FadeState         g_fadeState   = FADE_NONE;
DRAM_ATTR static int               g_fadePos     = 0;
DRAM_ATTR static float             g_fadeLastEnv  = 1.0f;
DRAM_ATTR static float             g_fadeStartEnv = 1.0f;

DRAM_ATTR static float g_lastOutSample[NUM_OUTPUTS] = {0, 0, 0, 0};

static inline void resetSignalPathState() {
    for (int i = 0; i < NUM_EQ_BANDS; i++) {
        for (int k = 0; k < 4; k++) {
            resetBiquad(bqSharedL[i][k]);
            resetBiquad(bqSharedR[i][k]);
        }
    }
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        for (int i = 0; i < MAX_OUT_BANDS; i++) {
            for (int k = 0; k < 4; k++) resetBiquad(bqOut[o][i][k]);
        }
        memset(delayBuf[o], 0, sizeof(delayBuf[o]));
        delayHead[o] = 0;
    }
}

static inline void requestFade() {
    g_fadeRequest++;
}

/* ==================================================================
 *  Coefficient computation
 * ================================================================== */
static inline double wn(float freq) {
    return 2.0 * M_PI * (double)freq / (double)SAMPLE_RATE_HZ;
}

static void coeffPeak(Biquad &b, float freq, float gainDb, float q) {
    double A = pow(10.0, (double)gainDb / 40.0), w = wn(freq);
    double cw = cos(w), alpha = sin(w) / (2.0 * q);
    double b0 = 1 + alpha * A, b1 = -2 * cw, b2 = 1 - alpha * A;
    double a0 = 1 + alpha / A;
    b.b0 = (float)(b0 / a0); b.b1 = (float)(b1 / a0); b.b2 = (float)(b2 / a0);
    b.a1 = (float)(-2 * cw / a0); b.a2 = (float)((1 - alpha / A) / a0);
}

static void coeffLowShelf(Biquad &b, float freq, float gainDb, float q) {
    double A = pow(10.0, (double)gainDb / 40.0), w = wn(freq);
    double cw = cos(w), sw = sin(w), a = sw / (2.0 * q), sA = sqrt(A);
    double b0 = A * ((A + 1) - (A - 1) * cw + 2 * sA * a);
    double b1 = 2 * A * ((A - 1) - (A + 1) * cw);
    double b2 = A * ((A + 1) - (A - 1) * cw - 2 * sA * a);
    double a0 = (A + 1) + (A - 1) * cw + 2 * sA * a;
    double a1 = -2 * ((A - 1) + (A + 1) * cw);
    double a2 = (A + 1) + (A - 1) * cw - 2 * sA * a;
    b.b0 = (float)(b0 / a0); b.b1 = (float)(b1 / a0); b.b2 = (float)(b2 / a0);
    b.a1 = (float)(a1 / a0); b.a2 = (float)(a2 / a0);
}

static void coeffHighShelf(Biquad &b, float freq, float gainDb, float q) {
    double A = pow(10.0, (double)gainDb / 40.0), w = wn(freq);
    double cw = cos(w), sw = sin(w), a = sw / (2.0 * q), sA = sqrt(A);
    double b0 = A * ((A + 1) + (A - 1) * cw + 2 * sA * a);
    double b1 = -2 * A * ((A - 1) + (A + 1) * cw);
    double b2 = A * ((A + 1) + (A - 1) * cw - 2 * sA * a);
    double a0 = (A + 1) - (A - 1) * cw + 2 * sA * a;
    double a1 = 2 * ((A - 1) - (A + 1) * cw);
    double a2 = (A + 1) - (A - 1) * cw - 2 * sA * a;
    b.b0 = (float)(b0 / a0); b.b1 = (float)(b1 / a0); b.b2 = (float)(b2 / a0);
    b.a1 = (float)(a1 / a0); b.a2 = (float)(a2 / a0);
}

static void coeffNotch(Biquad &b, float freq, float q) {
    double w = wn(freq), cw = cos(w), a = sin(w) / (2.0 * q), a0 = 1 + a;
    b.b0 = (float)(1 / a0); b.b1 = (float)(-2 * cw / a0); b.b2 = (float)(1 / a0);
    b.a1 = (float)(-2 * cw / a0); b.a2 = (float)((1 - a) / a0);
}

static void coeffBypass(Biquad &b) {
    b.b0 = 1; b.b1 = 0; b.b2 = 0; b.a1 = 0; b.a2 = 0;
}

static void coeff1stLPF(Biquad &b, float freq) {
    double K = tan(M_PI * (double)freq / (double)SAMPLE_RATE_HZ);
    double n = 1.0 / (1.0 + K);
    b.b0 = (float)(K * n); b.b1 = (float)(K * n); b.b2 = 0;
    b.a1 = (float)((K - 1) / (K + 1)); b.a2 = 0;
}
static void coeff1stHPF(Biquad &b, float freq) {
    double K = tan(M_PI * (double)freq / (double)SAMPLE_RATE_HZ);
    double n = 1.0 / (1.0 + K);
    b.b0 = (float)(n); b.b1 = (float)(-n); b.b2 = 0;
    b.a1 = (float)((K - 1) / (K + 1)); b.a2 = 0;
}
/* 2nd-order LPF/HPF with explicit Q. The plain coeff2ndLPF/HPF wrappers
 * use Butterworth Q (0.7071). Higher-order Linkwitz-Riley cascades use the
 * Butterworth Q-pairs below. */
static void coeff2ndLPF_Q(Biquad &b, float freq, double Q) {
    double w = wn(freq), cw = cos(w), a = sin(w) / (2.0 * Q), a0 = 1 + a;
    b.b0 = (float)((1 - cw) / (2 * a0));
    b.b1 = (float)((1 - cw) / a0);
    b.b2 = (float)((1 - cw) / (2 * a0));
    b.a1 = (float)(-2 * cw / a0); b.a2 = (float)((1 - a) / a0);
}
static void coeff2ndHPF_Q(Biquad &b, float freq, double Q) {
    double w = wn(freq), cw = cos(w), a = sin(w) / (2.0 * Q), a0 = 1 + a;
    b.b0 = (float)((1 + cw) / (2 * a0));
    b.b1 = (float)(-(1 + cw) / a0);
    b.b2 = (float)((1 + cw) / (2 * a0));
    b.a1 = (float)(-2 * cw / a0); b.a2 = (float)((1 - a) / a0);
}
static inline void coeff2ndLPF(Biquad &b, float freq) { coeff2ndLPF_Q(b, freq, 0.70710678); }
static inline void coeff2ndHPF(Biquad &b, float freq) { coeff2ndHPF_Q(b, freq, 0.70710678); }

/* Butterworth section Q values, used to build Linkwitz-Riley filters as a
 * cascade of two equal-order Butterworth filters:
 *   LR4 (24 dB/oct) = two 2nd-order Butterworth (Q = 0.7071 each)
 *   LR8 (48 dB/oct) = two 4th-order Butterworth; each 4th-order Butterworth
 *                     is a 0.5412 + 1.3066 Q pair, so LR8 = these four Qs. */
static const double LR8_Q[4] = { 0.54119610, 1.30656296, 0.54119610, 1.30656296 };

static int computeBandCoeffs(Biquad stages[4], int type, float freq,
                             float gainDb, float q, int steepness) {
    switch (type) {
        case FTYPE_PEAK:       coeffPeak     (stages[0], freq, gainDb, q); return 1;
        case FTYPE_LOW_SHELF:  coeffLowShelf (stages[0], freq, gainDb, q); return 1;
        case FTYPE_HIGH_SHELF: coeffHighShelf(stages[0], freq, gainDb, q); return 1;
        case FTYPE_NOTCH:      coeffNotch    (stages[0], freq, q);         return 1;
        case FTYPE_HIGH_CUT: {
            if (steepness == 6)  { coeff1stLPF(stages[0], freq); return 1; }
            if (steepness == 12) { coeff2ndLPF(stages[0], freq); return 1; }
            if (steepness == 24) {                       /* LR4 = 2x BW Q=0.7071 */
                coeff2ndLPF(stages[0], freq); stages[1] = stages[0]; return 2;
            }
            for (int k = 0; k < 4; k++)                  /* LR8 = 4x BW Q-pair */
                coeff2ndLPF_Q(stages[k], freq, LR8_Q[k]);
            return 4;
        }
        case FTYPE_LOW_CUT: {
            if (steepness == 6)  { coeff1stHPF(stages[0], freq); return 1; }
            if (steepness == 12) { coeff2ndHPF(stages[0], freq); return 1; }
            if (steepness == 24) {                       /* LR4 = 2x BW Q=0.7071 */
                coeff2ndHPF(stages[0], freq); stages[1] = stages[0]; return 2;
            }
            for (int k = 0; k < 4; k++)                  /* LR8 = 4x BW Q-pair */
                coeff2ndHPF_Q(stages[k], freq, LR8_Q[k]);
            return 4;
        }
        default: coeffBypass(stages[0]); return 1;
    }
}

/* Staging buffers: coefficients are computed here (heavy double-precision
 * trig) OUTSIDE the DSP lock, then copied into the live bq* arrays under the
 * lock by the commit* helpers (cheap, no trig). This keeps the audio task
 * from stalling on coefficient math while the lock is held. The staging
 * buffers are only touched by setup() and the /set handler, which never run
 * concurrently, so they need no lock of their own. Shared coeffs are identical
 * for L and R, so only one staged set is kept and copied to both on commit. */
DRAM_ATTR static Biquad stgShared[NUM_EQ_BANDS][4];
DRAM_ATTR static int    stgSharedStages[NUM_EQ_BANDS];
DRAM_ATTR static int    stgActiveSharedBands[NUM_EQ_BANDS];
DRAM_ATTR static int    stgActiveSharedCount = 0;

DRAM_ATTR static Biquad stgOut[NUM_OUTPUTS][MAX_OUT_BANDS][4];
DRAM_ATTR static int    stgOutStages[NUM_OUTPUTS][MAX_OUT_BANDS];
DRAM_ATTR static int    stgActiveOutBands[NUM_OUTPUTS][MAX_OUT_BANDS];
DRAM_ATTR static int    stgActiveOutCount[NUM_OUTPUTS] = {0};

/* Compute shared-EQ coefficients from the global sharedEQ[] settings into the
 * staging buffers. Call OUTSIDE the lock. (Parameterless on purpose: an .ino
 * function taking a sketch-defined struct type would get an auto-generated
 * prototype placed above the struct's definition and fail to compile.) */
static void stageSharedEQ() {
    int cnt = 0;
    for (int i = 0; i < NUM_EQ_BANDS; i++) {
        if (!sharedEQ[i].active || sharedEQ[i].type == FTYPE_BYPASS) {
            coeffBypass(stgShared[i][0]); resetBiquad(stgShared[i][0]);
            stgSharedStages[i] = 1; continue;
        }
        Biquad tmp[4];
        int n = computeBandCoeffs(tmp, sharedEQ[i].type, sharedEQ[i].freq,
                                  sharedEQ[i].gain, sharedEQ[i].q, sharedEQ[i].steepness);
        stgSharedStages[i] = n;
        for (int k = 0; k < n; k++) { stgShared[i][k] = tmp[k]; resetBiquad(stgShared[i][k]); }
        stgActiveSharedBands[cnt++] = i;
    }
    stgActiveSharedCount = cnt;
}

/* Copy staged shared-EQ coefficients into the live arrays. Call UNDER lock. */
static void commitSharedEQ() {
    for (int i = 0; i < NUM_EQ_BANDS; i++)
        for (int k = 0; k < 4; k++) { bqSharedL[i][k] = stgShared[i][k]; bqSharedR[i][k] = stgShared[i][k]; }
    memcpy(sharedEQStages,    stgSharedStages,      sizeof(sharedEQStages));
    memcpy(activeSharedBands, stgActiveSharedBands, sizeof(activeSharedBands));
    activeSharedCount = stgActiveSharedCount;
}

/* Compute per-output coefficients from the global outBands[]/outBandCount[]
 * settings into the staging buffers. Call OUTSIDE the lock. Parameterless for
 * the same .ino-prototype reason as stageSharedEQ(). */
static void stageOutBands() {
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        int c = 0;
        for (int i = 0; i < outBandCount[o]; i++) {
            if (!outBands[o][i].on || !outBands[o][i].active ||
                outBands[o][i].type == FTYPE_BYPASS) {
                coeffBypass(stgOut[o][i][0]); resetBiquad(stgOut[o][i][0]);
                stgOutStages[o][i] = 1; continue;
            }
            Biquad otmp[4];
            int n = computeBandCoeffs(otmp, outBands[o][i].type, outBands[o][i].freq,
                                      outBands[o][i].gain, outBands[o][i].q,
                                      outBands[o][i].steepness);
            stgOutStages[o][i] = n;
            for (int k = 0; k < n; k++) { stgOut[o][i][k] = otmp[k]; resetBiquad(stgOut[o][i][k]); }
            stgActiveOutBands[o][c++] = i;
        }
        stgActiveOutCount[o] = c;
    }
}

/* Copy staged per-output coefficients into the live arrays. Call UNDER lock. */
static void commitOutBands() {
    memcpy(bqOut,          stgOut,            sizeof(bqOut));
    memcpy(outBandStages,  stgOutStages,      sizeof(outBandStages));
    memcpy(activeOutBands, stgActiveOutBands, sizeof(activeOutBands));
    for (int o = 0; o < NUM_OUTPUTS; o++) activeOutCount[o] = stgActiveOutCount[o];
}

/* ==================================================================
 *  IDF I2C helpers
 * ================================================================== */
static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(dev, buf, 2, 100);
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *val) {
    return i2c_master_transmit_receive(dev, &reg, 1, val, 1, 100);
}

#define I2C_W(dev, r, v) do {                                                 \
    esp_err_t _e = i2c_write_reg((dev), (r), (v));                            \
    if (_e != ESP_OK) {                                                       \
        Serial.printf("[I2C] write 0x%02X=0x%02X failed: %s\n",               \
                      (r), (v), esp_err_to_name(_e));                         \
        triggerError();                                                       \
        return _e;                                                            \
    }                                                                         \
} while (0)

static void i2c_scan_print() {
    Serial.println("=== I2C SCAN ===");
    for (uint8_t addr = 1; addr < 127; addr++) {
        if (i2c_master_probe(s_i2c_bus, addr, 50) == ESP_OK) {
            Serial.printf("  Found device at 0x%02X\n", addr);
        }
    }
    Serial.println("=== END SCAN ===");
}

/* ==================================================================
 *  Status LED helpers
 * ================================================================== */
static void statusLedSetRaw(uint32_t color) {
    if (color == g_lastLedColorWritten) return;
    g_lastLedColorWritten = color;
    statusLed.setPixelColor(0, color);
    statusLed.show();
}

/* Compute volume-mode LED color.
 *
 * Maps volume 0..1 onto a smooth green -> yellow -> red gradient.
 * At exactly 100% (vol >= 0.999) the LED flashes red at 4 Hz.
 *
 * Color ramp:
 *   0%   -> green  RGB(0, 200, 0)
 *   50%  -> yellow RGB(200, 200, 0)
 *   100% -> red    RGB(200, 0, 0)
 *
 * The ramp is piecewise linear in two segments:
 *   Segment A (0..50%): R rises 0->200, G stays 200
 *   Segment B (50..100%): G falls 200->0, R stays 200
 */
static uint32_t volumeLedColor(float vol) {
    /* At 100%: flashing red */
    if (vol >= 0.999f) {
        if ((int32_t)(millis() - g_volFlashNextMs) >= 0) {
            g_volFlashOn      = !g_volFlashOn;
            g_volFlashNextMs  = millis() + VOL_FLASH_PERIOD_MS;
        }
        return g_volFlashOn ? STATUS_COLOR_ERROR : STATUS_COLOR_BLACK;
    }

    /* Clamp to [0, 1] */
    if (vol < 0.0f) vol = 0.0f;
    if (vol > 1.0f) vol = 1.0f;

    uint8_t r, g;
    if (vol <= 0.5f) {
        /* 0..50%: green -> yellow */
        float t = vol / 0.5f;          /* 0..1 */
        r = (uint8_t)(t * 200.0f);
        g = 200;
    } else {
        /* 50..100%: yellow -> red */
        float t = (vol - 0.5f) / 0.5f; /* 0..1 */
        r = 200;
        g = (uint8_t)((1.0f - t) * 200.0f);
    }
    return RGB(r, g, 0);
}

/* Compute the color that should be displayed right now. */
static uint32_t statusComputeColor() {
    /* Volume mode overrides everything except fatal error */
    if (!g_errorFatal && g_volumeMode) {
        return volumeLedColor(volume);
    }

    if (g_errorFatal) return STATUS_COLOR_ERROR;
    if (g_errorUntilMs != 0) {
        if ((int32_t)(millis() - g_errorUntilMs) < 0) {
            return STATUS_COLOR_ERROR;
        }
        g_errorUntilMs = 0;
    }
    return STATUS_COLOR_READY;
}

static void statusLedRefresh() {
    /* Volume mode: always call setPixelColor because the flash state
     * can change even if the color didn't (dedup breaks flashing).
     * For all other modes: dedup is safe. */
    if (g_volumeMode && !g_errorFatal) {
        uint32_t c = statusComputeColor();
        /* Force write — dedup bypassed so the flash toggle is visible */
        g_lastLedColorWritten = 0xFFFFFFFF;
        statusLedSetRaw(c);
    } else {
        statusLedSetRaw(statusComputeColor());
    }
}

/* ==================================================================
 *  Error reporting
 * ================================================================== */
static void triggerError() {
    if (g_errorFatal) return;
    g_errorUntilMs = millis() + ERROR_FLASH_MS;
    if (g_errorUntilMs == 0) g_errorUntilMs = 1;
}

static void triggerFatalError() {
    g_errorFatal = true;
}

static void fatalSpin(const char *msg) {
    Serial.print("FATAL: ");
    Serial.println(msg);
    triggerFatalError();
    statusLedRefresh();
    while (1) delay(1000);
}

/* ==================================================================
 *  Encoder ISR — CHANGE on both A and B, full quadrature debounce
 * ================================================================== */
#define ENC_DETENT_STEPS 4   /* quadrature transitions per mechanical detent */

static void IRAM_ATTR encoderISR() {
    /* gpio_get_level() is IRAM-safe; digitalRead() is not guaranteed to be. */
    int a = gpio_get_level((gpio_num_t)PIN_ENC_A);
    int b = gpio_get_level((gpio_num_t)PIN_ENC_B);
    int8_t curState = (int8_t)((a << 1) | b);

    int8_t delta = ENC_TABLE[(g_encState << 2) | curState];
    g_encState   = curState;

    if (delta == 0) return;

    g_encAccum += delta;

    if (g_encAccum >= ENC_DETENT_STEPS) {
        g_encSteps++;
        g_encAccum = 0;
    } else if (g_encAccum <= -ENC_DETENT_STEPS) {
        g_encSteps--;
        g_encAccum = 0;
    }
}

static void requestSave();   /* forward declaration */

/* ==================================================================
 *  Button + encoder service — called once per loop() tick
 * ================================================================== */
static void serviceControls() {
    const uint32_t now = millis();

    /* ---- Button ---- */
    bool isLow = (digitalRead(PIN_BUTTON) == LOW);

    if (isLow && !g_btnWasLow) {
        g_btnPressStartMs = now;
        g_btnWasLow       = true;
    } else if (!isLow && g_btnWasLow) {
        uint32_t held = now - g_btnPressStartMs;
        g_btnWasLow   = false;

        if (held >= BTN_DEBOUNCE_MS) {
            g_volumeMode = !g_volumeMode;
            if (g_volumeMode) {
                g_volumeModeExitMs = now + VOL_MODE_TIMEOUT_MS;
                g_volFlashOn      = true;
                g_volFlashNextMs  = now + VOL_FLASH_PERIOD_MS;
                Serial.printf("[BTN] volume mode ON  (vol=%.0f%%)\n",
                              volume * 100.0f);
            } else {
                Serial.println("[BTN] volume mode OFF");
                requestSave();
            }
        }
    }

    /* ---- Encoder ---- */
    int steps = g_encSteps;
    if (steps != 0) {
        g_encSteps -= steps;

        if (g_volumeMode) {
            float newVol = volume + steps * VOL_STEP;
            if (newVol < 0.0f) newVol = 0.0f;
            if (newVol > 1.0f) newVol = 1.0f;

            xSemaphoreTake(dspMutex, portMAX_DELAY);
            volume = newVol;
            g_volGain = powf(newVol, 2.5f);
            requestFade();
            xSemaphoreGive(dspMutex);

            g_volumeModeExitMs = now + VOL_MODE_TIMEOUT_MS;

            Serial.printf("[ENC] vol -> %.0f%%\n", newVol * 100.0f);
        }
    }

    /* ---- Volume mode auto-exit ---- */
    if (g_volumeMode && (int32_t)(now - g_volumeModeExitMs) >= 0) {
        g_volumeMode = false;
        Serial.println("[VOL] auto-exit volume mode");
        requestSave();
    }
}

/* ==================================================================
 *  Hardware — PDN
 * ================================================================== */
static esp_err_t amp_pdn_init_off() {
    gpio_config_t pdn_gpio = {
        .pin_bit_mask = 1ULL << PIN_AMP_PDN,
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    esp_err_t e = gpio_config(&pdn_gpio);
    if (e != ESP_OK) return e;
    gpio_set_level(PIN_AMP_PDN, AMP_PDN_OFF);
    return ESP_OK;
}

/* ==================================================================
 *  I2C bus init
 * ================================================================== */
static esp_err_t i2c_bus_init() {
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port                     = I2C_NUM_0,
        .sda_io_num                   = PIN_I2C_SDA,
        .scl_io_num                   = PIN_I2C_SCL,
        .clk_source                   = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt            = 7,
        .flags = { .enable_internal_pullup = true },
    };
    esp_err_t e = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (e != ESP_OK) { Serial.printf("[I2C] bus init: %s\n", esp_err_to_name(e)); return e; }

    i2c_device_config_t common_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = 0,
        .scl_speed_hz    = I2C_FREQ_HZ,
    };
    i2c_device_config_t cfg;

    cfg = common_cfg; cfg.device_address = ADC_I2C_ADDR;
    e = i2c_master_bus_add_device(s_i2c_bus, &cfg, &s_adc_dev);
    if (e != ESP_OK) return e;

    cfg = common_cfg; cfg.device_address = AMP1_I2C_ADDR;
    e = i2c_master_bus_add_device(s_i2c_bus, &cfg, &s_amp1_dev);
    if (e != ESP_OK) return e;

    cfg = common_cfg; cfg.device_address = AMP2_I2C_ADDR;
    e = i2c_master_bus_add_device(s_i2c_bus, &cfg, &s_amp2_dev);
    if (e != ESP_OK) return e;

    return ESP_OK;
}

/* ==================================================================
 *  I2S init — dual standard stereo
 *
 *  I2S0: full-duplex MASTER. Drives the shared BCLK (GPIO4) + WS (GPIO5).
 *        RX <- ADC (GPIO7), TX -> AMP1 (GPIO6). Default PLL clock source.
 *  I2S1: TX-only SLAVE. Reads the SAME BCLK/WS pins as inputs (a pad can
 *        drive one peripheral's output and feed other peripherals' inputs
 *        at the same time) and clocks AMP2 (GPIO21). Being slaved to I2S0
 *        keeps both amps sample-rate-locked.
 *
 *  The shared-clock slave is confirmed working: I2S1 reads GPIO4/5 as inputs
 *  without any manual GPIO re-routing. If a future core regression breaks the
 *  slave clock, the MANUAL-INPUT fallback (connect_in_signal) is noted below.
 * ================================================================== */
static esp_err_t i2s_std_init() {
    /* ---------- I2S0: master, full-duplex ---------- */
    i2s_chan_config_t chan0 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
    chan0.dma_desc_num  = 4;
    chan0.dma_frame_num = 16;
    chan0.auto_clear    = true;

    esp_err_t e = i2s_new_channel(&chan0, &s_tx0_handle, &s_rx_handle);
    if (e != ESP_OK) { Serial.printf("[I2S0] new_channel: %s\n", esp_err_to_name(e)); return e; }

    i2s_std_config_t std0 = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,
            .ws   = PIN_I2S_FSYNC,
            .dout = PIN_I2S_DOUT,
            .din  = PIN_I2S_DIN,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    /* Default clock source (PLL) — same as the test code. APLL isn't exposed
     * in this core build (I2S_CLK_SRC_APLL undeclared), and it isn't needed
     * for the framing change; standard stereo removes the slot ambiguity on
     * its own. mclk_multiple 256 is already the std default. */

    e = i2s_channel_init_std_mode(s_tx0_handle, &std0);
    if (e != ESP_OK) { Serial.printf("[I2S0] init tx: %s\n", esp_err_to_name(e)); return e; }
    e = i2s_channel_init_std_mode(s_rx_handle, &std0);
    if (e != ESP_OK) { Serial.printf("[I2S0] init rx: %s\n", esp_err_to_name(e)); return e; }

    /* ---------- I2S1: slave, TX only ---------- */
    i2s_chan_config_t chan1 = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_1, I2S_ROLE_SLAVE);
    chan1.dma_desc_num  = 4;
    chan1.dma_frame_num = 16;
    chan1.auto_clear    = true;

    e = i2s_new_channel(&chan1, &s_tx1_handle, NULL);
    if (e != ESP_OK) { Serial.printf("[I2S1] new_channel: %s\n", esp_err_to_name(e)); return e; }

    i2s_std_config_t std1 = {
        .clk_cfg  = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE_HZ),
        .slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
                        I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = PIN_I2S_BCLK,    /* shared — consumed as input by the slave */
            .ws   = PIN_I2S_FSYNC,   /* shared — consumed as input by the slave */
            .dout = PIN_I2S_DOUT2,
            .din  = I2S_GPIO_UNUSED,
            .invert_flags = { .mclk_inv = false, .bclk_inv = false, .ws_inv = false },
        },
    };
    e = i2s_channel_init_std_mode(s_tx1_handle, &std1);
    if (e != ESP_OK) {
        /* If this errors with "GPIO already in use", the driver refuses to
         * share GPIO4/5. MANUAL-INPUT fallback: set std1.gpio_cfg.bclk and
         * .ws to I2S_GPIO_UNUSED above, re-init, then wire the clock inputs
         * by hand here:
         *   esp_rom_gpio_connect_in_signal(PIN_I2S_BCLK,  I2S1I_BCK_IN_IDX, false);
         *   esp_rom_gpio_connect_in_signal(PIN_I2S_FSYNC, I2S1I_WS_IN_IDX,  false);
         *   gpio_set_direction(PIN_I2S_BCLK,  GPIO_MODE_INPUT_OUTPUT);
         *   gpio_set_direction(PIN_I2S_FSYNC, GPIO_MODE_INPUT_OUTPUT);
         */
        Serial.printf("[I2S1] init tx: %s\n", esp_err_to_name(e));
        return e;
    }

    /* NOTE: previously this re-asserted I2S0's BCLK/WS output onto GPIO4/5 by
     * hand (esp_rom_gpio_connect_out_signal + INPUT_OUTPUT). That hand-routing
     * was killing the clock for both amps, so it's removed — we let the driver
     * own the pads. If the SLAVE (I2S1) turns out not to see the clock, the fix
     * is the manual-INPUT path (connect_in_signal), NOT re-driving the output. */

    /* Slave armed first (waits for clock), then master starts the clocks. */
    e = i2s_channel_enable(s_tx1_handle);
    if (e != ESP_OK) { Serial.printf("[I2S1] enable: %s\n", esp_err_to_name(e)); return e; }
    e = i2s_channel_enable(s_rx_handle);
    if (e != ESP_OK) { Serial.printf("[I2S0] enable rx: %s\n", esp_err_to_name(e)); return e; }
    e = i2s_channel_enable(s_tx0_handle);
    if (e != ESP_OK) { Serial.printf("[I2S0] enable tx: %s\n", esp_err_to_name(e)); return e; }

    return ESP_OK;
}

/* ==================================================================
 *  TLV320ADC6120 init — standard stereo I2S, 32-bit
 * ================================================================== */
static esp_err_t adc_init() {
    I2C_W(s_adc_dev, ADC_REG_PAGE_SEL, 0x00);

    {
        esp_err_t e = ESP_FAIL;
        for (int attempt = 0; attempt < 3 && e != ESP_OK; attempt++) {
            e = i2c_write_reg(s_adc_dev, ADC_REG_SLEEP_CFG, 0x81);
            if (e != ESP_OK) vTaskDelay(pdMS_TO_TICKS(2));
        }
        if (e != ESP_OK) return e;
    }
    vTaskDelay(pdMS_TO_TICKS(2));

    /* ASI_CFG0 = 0x70 : bits[7:6]=01 (I2S), bits[5:4]=11 (32-bit), default
     * FSYNC/BCLK polarity.  (Was 0x30 = TDM, 32-bit.) */
    I2C_W(s_adc_dev, ADC_REG_ASI_CFG0,      0x70);
    I2C_W(s_adc_dev, ADC_REG_ASI_CFG1,      0x00);
    I2C_W(s_adc_dev, ADC_REG_ASI_CFG2,      0x00);
    /* Channel-to-slot: ch1 -> slot 0 (left), ch2 -> slot 1 (right). In I2S
     * mode these map to the two halves of the stereo frame. */
    I2C_W(s_adc_dev, ADC_REG_ASI_CH1,       0x00);
    I2C_W(s_adc_dev, ADC_REG_ASI_CH2,       0x01);
    I2C_W(s_adc_dev, ADC_REG_MST_CFG0,      0x00);   /* slave (ESP is master) */
    I2C_W(s_adc_dev, ADC_REG_DSP_CFG0,      0x00);
    I2C_W(s_adc_dev, ADC_REG_IN_CH_EN,      0xC0);
    I2C_W(s_adc_dev, ADC_REG_ASI_OUT_CH_EN, 0xC0);
    I2C_W(s_adc_dev, ADC_REG_PWR_CFG,       0xE0);

    vTaskDelay(pdMS_TO_TICKS(10));
    return ESP_OK;
}

/* ==================================================================
 *  TAS5827 — Phase 1: configure one amp (standard I2S, stays in HIZ)
 *
 *  Both amps now get identical config — standard stereo I2S, no slot
 *  offset (the offset machinery was only needed for the shared TDM frame).
 * ================================================================== */
static esp_err_t amp_configure_one(i2c_master_dev_handle_t dev,
                                   uint8_t i2c_addr,
                                   const char *amp_tag) {
    if (i2c_master_probe(s_i2c_bus, i2c_addr, 100) != ESP_OK) {
        Serial.printf("[%s] NACK at 0x%02X\n", amp_tag, i2c_addr);
        triggerError();
        return ESP_FAIL;
    }

    I2C_W(dev, TAS_REG_PAGE_SEL,     0x00);
    I2C_W(dev, TAS_REG_BOOK_SEL,     0x00);
    I2C_W(dev, TAS_REG_DEVICE_CTRL2, DCTRL2_MODE_HIZ);
    I2C_W(dev, TAS_REG_RESET,        0x11);
    vTaskDelay(pdMS_TO_TICKS(10));
    I2C_W(dev, TAS_REG_PAGE_SEL,     0x00);
    I2C_W(dev, TAS_REG_BOOK_SEL,     0x00);
    I2C_W(dev, TAS_REG_DEVICE_CTRL2, DCTRL2_MODE_HIZ);
    vTaskDelay(pdMS_TO_TICKS(5));

    /* SAP_CTRL1 = 0x03 : bits[5:4]=00 (standard I2S), bits[1:0]=11 (32-bit).
     *   (Was 0x13 = TDM, 32-bit.)
     * SAP_CTRL2 = 0x00 : no data offset (standard I2S framing).
     *   (Was the per-amp TDM slot offset.)
     * SAP_CTRL3 = 0x11 : left/right channel map — UNVERIFIED for I2S. If a
     *   channel comes out swapped or doubled, this is the register to adjust
     *   against the TAS5827 datasheet. */
    I2C_W(dev, TAS_REG_SAP_CTRL1, 0x03);
    I2C_W(dev, TAS_REG_SAP_CTRL2, 0x00);
    I2C_W(dev, TAS_REG_SAP_CTRL3, 0x11);

    I2C_W(dev, TAS_REG_DEVICE_CTRL1, 0x00);
    I2C_W(dev, TAS_REG_DIG_VOL,      0x30);
    I2C_W(dev, TAS_REG_AGAIN,        0x00);

    /* Stays in HIZ — amp_enable_one() moves it to PLAY after settle. */
    return ESP_OK;
}

/* ==================================================================
 *  TAS5827 — Phase 2: wait for clock lock, then go to PLAY
 * ================================================================== */
static esp_err_t amp_enable_one(i2c_master_dev_handle_t dev,
                                const char *amp_tag) {
    uint8_t fault = 0xFF;
    for (int i = 0; i < 30; i++) {
        if (i2c_read_reg(dev, TAS_REG_GLOBAL_FAULT1, &fault) == ESP_OK
            && (fault & 0x04) == 0) break;
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    if (fault & 0x04) {
        Serial.printf("[%s] clock-detect did not lock (GLOBAL_FAULT1=0x%02X)\n",
                      amp_tag, fault);
        triggerError();
        return ESP_FAIL;
    }

    I2C_W(dev, TAS_REG_DEVICE_CTRL2, DCTRL2_MODE_PLAY);
    vTaskDelay(pdMS_TO_TICKS(5));
    Serial.printf("[%s] PLAY\n", amp_tag);
    return ESP_OK;
}

/* ==================================================================
 *  Amps init — Phase 1 only (configure, leave in HIZ)
 * ================================================================== */
static esp_err_t amps_init() {
    vTaskDelay(pdMS_TO_TICKS(5));
    gpio_set_level(PIN_AMP_PDN, AMP_PDN_ON);
    vTaskDelay(pdMS_TO_TICKS(15));     /* TAS5827 needs ~5 ms after PDN; 15 ms is safe */

    esp_err_t e = amp_configure_one(s_amp1_dev, AMP1_I2C_ADDR, "AMP1");
    if (e != ESP_OK) return e;
    e = amp_configure_one(s_amp2_dev, AMP2_I2C_ADDR, "AMP2");
    return e;
}

/* ==================================================================
 *  Amps enable — Phase 2 (clock-lock check + transition to PLAY)
 * ================================================================== */
static void amps_enable() {
    amp_enable_one(s_amp1_dev, "AMP1");
    amp_enable_one(s_amp2_dev, "AMP2");
}

/* ==================================================================
 *  TAS5827 fault watchdog + auto-recovery   (polled from loop())
 * ================================================================== */
#define AMP_FAULT_POLL_MS       250
#define AMP_RECOVER_MIN_GAP_MS  2000
#define AMP_RECOVER_MAX_TRIES   5

struct AmpFaultRec { uint32_t lastTryMs; uint8_t fails; bool gaveUp; uint8_t lastG1, lastG2; };
static AmpFaultRec s_ampFault[2] = {};

static void amp_fault_service() {
    static uint32_t lastPoll = 0;
    uint32_t now = millis();
    if (now - lastPoll < AMP_FAULT_POLL_MS) return;
    lastPoll = now;

    i2c_master_dev_handle_t dev[2] = { s_amp1_dev, s_amp2_dev };
    const char *tag[2] = { "AMP1", "AMP2" };

    for (int a = 0; a < 2; a++) {
        if (!dev[a]) continue;
        uint8_t chan = 0, g1 = 0, g2 = 0;
        i2c_read_reg(dev[a], TAS_REG_CHAN_FAULT,    &chan);
        i2c_read_reg(dev[a], TAS_REG_GLOBAL_FAULT1, &g1);
        i2c_read_reg(dev[a], TAS_REG_GLOBAL_FAULT2, &g2);

        if (g1 != s_ampFault[a].lastG1 || g2 != s_ampFault[a].lastG2) {
            if (g1 || g2 || s_ampFault[a].lastG1 || s_ampFault[a].lastG2)
                Serial.printf("[%s] global fault g1=0x%02X g2=0x%02X\n", tag[a], g1, g2);
            s_ampFault[a].lastG1 = g1;
            s_ampFault[a].lastG2 = g2;
        }

        if (chan == 0) {
            if (s_ampFault[a].fails || s_ampFault[a].gaveUp)
                Serial.printf("[%s] channel fault cleared\n", tag[a]);
            s_ampFault[a].fails  = 0;
            s_ampFault[a].gaveUp = false;
            continue;
        }

        if (s_ampFault[a].gaveUp) continue;
        if (now - s_ampFault[a].lastTryMs < AMP_RECOVER_MIN_GAP_MS) continue;
        s_ampFault[a].lastTryMs = now;

        if (++s_ampFault[a].fails > AMP_RECOVER_MAX_TRIES) {
            s_ampFault[a].gaveUp = true;
            Serial.printf("[%s] auto-recovery gave up after %d tries "
                          "(chan=0x%02X g1=0x%02X g2=0x%02X)\n",
                          tag[a], AMP_RECOVER_MAX_TRIES, chan, g1, g2);
            continue;
        }

        Serial.printf("[%s] CHAN_FAULT=0x%02X (g1=0x%02X g2=0x%02X) — recovering %d/%d (HIZ->PLAY)\n",
                      tag[a], chan, g1, g2, s_ampFault[a].fails, AMP_RECOVER_MAX_TRIES);
        requestFade();
        i2c_write_reg(dev[a], TAS_REG_DEVICE_CTRL2, DCTRL2_MODE_HIZ);
        vTaskDelay(pdMS_TO_TICKS(8));
        i2c_write_reg(dev[a], TAS_REG_DEVICE_CTRL2, DCTRL2_MODE_PLAY);
        vTaskDelay(pdMS_TO_TICKS(8));
    }
}

/* ==================================================================
 *  Audio task — standard stereo: 1 stereo RX, 2 stereo TX
 *
 *  Logical outputs 0..3 map to:
 *    out0 -> AMP1 left  (tx0 slot 0)
 *    out1 -> AMP1 right (tx0 slot 1)
 *    out2 -> AMP2 left  (tx1 slot 0)
 *    out3 -> AMP2 right (tx1 slot 1)
 * ================================================================== */
static void IRAM_ATTR audioTask(void* /*arg*/) {
    const size_t bytesPerFrame = STD_SLOTS * sizeof(int32_t);   /* stereo frame */
    const size_t bytesPerBuf   = BUFFER_FRAMES * bytesPerFrame;

    int32_t* rxBuf  = (int32_t*)heap_caps_malloc(bytesPerBuf, MALLOC_CAP_DMA);
    int32_t* tx0Buf = (int32_t*)heap_caps_malloc(bytesPerBuf, MALLOC_CAP_DMA);
    int32_t* tx1Buf = (int32_t*)heap_caps_malloc(bytesPerBuf, MALLOC_CAP_DMA);
    if (!rxBuf || !tx0Buf || !tx1Buf) {
        Serial.println("[AUDIO] DMA buffer alloc failed");
        triggerFatalError();
        statusLedRefresh();
        vTaskDelete(NULL);
        return;
    }
    vTaskDelay(pdMS_TO_TICKS(200));

    float denormal = 1e-18f;

    while (true) {
        size_t br = 0, bw = 0;
        esp_err_t err = i2s_channel_read(s_rx_handle, rxBuf, bytesPerBuf, &br, 100);
        if (err != ESP_OK || br == 0) continue;
        const size_t frames = br / bytesPerFrame;

        xSemaphoreTake(dspMutex, portMAX_DELAY);

        if (g_fadeRequest != g_fadeSeen) {
            g_fadeSeen     = g_fadeRequest;
            g_fadeState    = FADE_DOWN;
            g_fadePos      = 0;
            g_fadeStartEnv = g_fadeLastEnv;
            resetSignalPathState();
        }

        const float vol     = g_volGain;
        const int   nShared = activeSharedCount;
        int nOut[NUM_OUTPUTS];
        for (int o = 0; o < NUM_OUTPUTS; o++) nOut[o] = activeOutCount[o];

        float blockPeak[NUM_OUTPUTS] = {0, 0, 0, 0};
        bool  blockClip[NUM_OUTPUTS] = {false, false, false, false};

        for (size_t i = 0; i < frames; i++) {
            int32_t *rxFrame = &rxBuf[i * STD_SLOTS];
            int32_t *t0Frame = &tx0Buf[i * STD_SLOTS];
            int32_t *t1Frame = &tx1Buf[i * STD_SLOTS];

            float inL = (float)(rxFrame[0] >> PCM_SHIFT) * (1.0f / 8388608.0f) + denormal;
            float inR = (float)(rxFrame[1] >> PCM_SHIFT) * (1.0f / 8388608.0f) + denormal;

            float eqL = inL, eqR = inR;
            for (int bi = 0; bi < nShared; bi++) {
                const int b = activeSharedBands[bi];
                for (int k = 0; k < sharedEQStages[b]; k++) {
                    eqL = processBiquad(bqSharedL[b][k], eqL);
                    eqR = processBiquad(bqSharedR[b][k], eqR);
                }
            }

            float outSample[NUM_OUTPUTS];
            for (int o = 0; o < NUM_OUTPUTS; o++) {
                float x;
                switch (routing[o]) {
                    case 0:  x = eqL;  break;
                    case 1:  x = eqR;  break;
                    default: x = 0.0f; break;
                }
                for (int bi = 0; bi < nOut[o]; bi++) {
                    const int b = activeOutBands[o][bi];
                    for (int k = 0; k < outBandStages[o][b]; k++)
                        x = processBiquad(bqOut[o][b][k], x);
                }
                x  = processDelay(o, x);
                x *= outputGain[o] * vol;
                if (outputPhase[o]) x = -x;
                outSample[o] = x;
            }

            float env = 1.0f;
            bool useHeld = false;
            switch (g_fadeState) {
                case FADE_DOWN: {
                    float t = (float)g_fadePos / (float)FADE_SAMPLES;
                    env = g_fadeStartEnv * 0.5f * (1.0f + cosf((float)M_PI * t));
                    useHeld = true;
                    if (++g_fadePos >= FADE_SAMPLES) {
                        resetSignalPathState();
                        g_fadeState = FADE_UP;
                        g_fadePos   = 0;
                        env         = 0.0f;
                        useHeld     = false;
                    }
                    break;
                }
                case FADE_UP: {
                    float t = (float)g_fadePos / (float)FADE_SAMPLES;
                    env = 0.5f * (1.0f - cosf((float)M_PI * t));
                    if (++g_fadePos >= FADE_SAMPLES) {
                        g_fadeState = FADE_NONE;
                        g_fadePos   = 0;
                        env         = 1.0f;
                    }
                    break;
                }
                default:
                    env = 1.0f;
                    break;
            }
            g_fadeLastEnv = env;

            for (int o = 0; o < NUM_OUTPUTS; o++) {
                float src = useHeld ? g_lastOutSample[o] : outSample[o];
                if (g_fadeState == FADE_NONE) {
                    g_lastOutSample[o] = outSample[o];
                }
                float s = src * env;

                float a = s < 0 ? -s : s;
                if (a > blockPeak[o]) blockPeak[o] = a;
                if (a >= METER_CLIP_THRESH) blockClip[o] = true;

                if (s >  1.0f) s =  1.0f;
                if (s < -1.0f) s = -1.0f;

                int32_t packed = (int32_t)(s * 8388607.0f) << 8;
                if (o < 2) t0Frame[o]       = packed;   /* AMP1 L/R */
                else       t1Frame[o - 2]   = packed;   /* AMP2 L/R */
            }
        }

        {
            uint32_t now = millis();
            for (int o = 0; o < NUM_OUTPUTS; o++) {
                float decayed = g_meterPeak[o] * METER_DECAY;
                if (blockPeak[o] > decayed) decayed = blockPeak[o];
                g_meterPeak[o] = decayed;
                if (blockClip[o]) {
                    g_meterClipUntilMs[o] = now + METER_CLIP_HOLD_MS;
                }
            }
        }

        xSemaphoreGive(dspMutex);
        denormal = -denormal;

        /* Both TX channels are clocked off the same shared BCLK/WS, so equal
         * frame counts keep AMP1 and AMP2 sample-aligned. */
        i2s_channel_write(s_tx0_handle, tx0Buf, br, &bw, 100);
        i2s_channel_write(s_tx1_handle, tx1Buf, br, &bw, 100);
        (void)bw;
    }
}

/* ==================================================================
 *  NVS save / load
 * ================================================================== */
struct EQBandPOD {
    float    freq, gain, q;
    int      type, steepness;
    bool     active;
    uint8_t  _pad[3];
};
static_assert(sizeof(EQBandPOD) == 24, "EQBandPOD size mismatch");

struct OutBandPOD {
    float    freq, gain, q;
    int      type, steepness;
    bool     active, on;
    uint8_t  _pad[2];
};
static_assert(sizeof(OutBandPOD) == 24, "OutBandPOD size mismatch");

struct MetaPOD {
    float    vol;
    int      routing[NUM_OUTPUTS];
    int      delays[NUM_OUTPUTS];
    float    outGain[NUM_OUTPUTS];
    uint8_t  outPhase[NUM_OUTPUTS];
    uint32_t magic;
};
static const uint32_t NVS_MAGIC = 0xD5047043UL;

#define SAVE_DEFER_MS  3000

DRAM_ATTR static volatile uint32_t g_savePendingDeadlineMs = 0;

static inline void requestSave() {
    uint32_t deadline = millis() + SAVE_DEFER_MS;
    if (deadline == 0) deadline = 1;
    g_savePendingDeadlineMs = deadline;
}

static void saveSettings();   /* forward declaration */

static TaskHandle_t s_saveTask = NULL;

static void saveTask(void* /*p*/) {
    while (true) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        saveSettings();
    }
}

static void serviceDeferredSave() {
    uint32_t deadline = g_savePendingDeadlineMs;
    if (deadline == 0) return;
    if ((int32_t)(millis() - deadline) < 0) return;
    g_savePendingDeadlineMs = 0;
    if (s_saveTask) xTaskNotifyGive(s_saveTask);
    else            saveSettings();
}

static void saveSettings() {
    MetaPOD meta;
    EQBandPOD eqPod[NUM_EQ_BANDS];
    int s_outBandCount[NUM_OUTPUTS];
    OutBand s_outBands[NUM_OUTPUTS][MAX_OUT_BANDS];

    xSemaphoreTake(dspMutex, portMAX_DELAY);
    meta.vol   = volume;
    meta.magic = NVS_MAGIC;
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        meta.routing[o]  = routing[o];
        meta.delays[o]   = delayInSamples[o];
        meta.outGain[o]  = outputGain[o];
        meta.outPhase[o] = outputPhase[o] ? 1 : 0;
    }
    for (int i = 0; i < NUM_EQ_BANDS; i++) {
        eqPod[i] = {sharedEQ[i].freq, sharedEQ[i].gain, sharedEQ[i].q,
                    sharedEQ[i].type, sharedEQ[i].steepness, sharedEQ[i].active, {0,0,0}};
    }
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        s_outBandCount[o] = outBandCount[o];
        for (int i = 0; i < MAX_OUT_BANDS; i++) {
            s_outBands[o][i] = outBands[o][i];
        }
    }
    xSemaphoreGive(dspMutex);

    prefs.begin("dsp4tdm", false);
    bool ok = true;
    ok &= (prefs.putBytes("meta", &meta, sizeof(meta)) == sizeof(meta));
    ok &= (prefs.putBytes("seq",  eqPod, sizeof(eqPod)) == sizeof(eqPod));

    for (int o = 0; o < NUM_OUTPUTS; o++) {
        int cnt = s_outBandCount[o];
        size_t blobSz = 1 + cnt * sizeof(OutBandPOD);
        uint8_t *blob = (uint8_t*)alloca(blobSz);
        blob[0] = (uint8_t)cnt;
        OutBandPOD *pods = (OutBandPOD*)(blob + 1);
        for (int i = 0; i < cnt; i++) {
            pods[i] = {s_outBands[o][i].freq, s_outBands[o][i].gain, s_outBands[o][i].q,
                       s_outBands[o][i].type, s_outBands[o][i].steepness,
                       s_outBands[o][i].active, s_outBands[o][i].on, {0,0}};
        }
        char key[4]; snprintf(key, sizeof(key), "ob%d", o);
        ok &= (prefs.putBytes(key, blob, blobSz) == blobSz);
    }
    prefs.end();
    if (!ok) { Serial.println("[NVS] save failed"); triggerError(); }
}

/* Clamp a stored float into range, substituting a default for NaN/Inf. */
static inline float sanitizeF(float v, float lo, float hi, float def) {
    if (!isfinite(v)) return def;
    return v < lo ? lo : (v > hi ? hi : v);
}

static void loadSettings() {
    prefs.begin("dsp4tdm", true);
    MetaPOD meta;
    size_t got = prefs.getBytes("meta", &meta, sizeof(meta));
    if (got != sizeof(meta) || meta.magic != NVS_MAGIC) {
        prefs.end();
        return;
    }
    volume = constrain(meta.vol, 0.0f, 1.0f);
    g_volGain = powf(volume, 2.5f);
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        routing[o]        = constrain(meta.routing[o], -1, 1);
        delayInSamples[o] = constrain(meta.delays[o], 0, DELAY_MAX_SAMP - 1);
        outputGain[o]     = constrain(meta.outGain[o], 0.0f, 1.0f);
        outputPhase[o]    = meta.outPhase[o] != 0;
    }

    EQBandPOD eqPod[NUM_EQ_BANDS];
    got = prefs.getBytes("seq", eqPod, sizeof(eqPod));
    if (got == sizeof(eqPod)) {
        for (int i = 0; i < NUM_EQ_BANDS; i++) {
            sharedEQ[i] = {
                sanitizeF(eqPod[i].freq,  20.0f, 20000.0f, 1000.0f),
                sanitizeF(eqPod[i].gain, -18.0f, 18.0f,    0.0f),
                sanitizeF(eqPod[i].q,      0.1f, 10.0f,    0.7f),
                constrain(eqPod[i].type,      0, 6),
                constrain(eqPod[i].steepness, 6, 48),
                eqPod[i].active };
        }
    }

    for (int o = 0; o < NUM_OUTPUTS; o++) {
        char key[4]; snprintf(key, sizeof(key), "ob%d", o);
        size_t maxSz = 1 + MAX_OUT_BANDS * sizeof(OutBandPOD);
        uint8_t *blob = (uint8_t*)alloca(maxSz);
        got = prefs.getBytes(key, blob, maxSz);
        if (got >= 1) {
            int cnt = constrain((int)blob[0], 0, MAX_OUT_BANDS);
            if (got < (size_t)(1 + cnt * (int)sizeof(OutBandPOD))) cnt = 0;
            outBandCount[o] = cnt;
            OutBandPOD *pods = (OutBandPOD*)(blob + 1);
            for (int i = 0; i < cnt; i++) {
                outBands[o][i] = {
                    sanitizeF(pods[i].freq,  20.0f, 20000.0f, 1000.0f),
                    sanitizeF(pods[i].gain, -18.0f, 18.0f,    0.0f),
                    sanitizeF(pods[i].q,      0.1f, 10.0f,    0.7f),
                    constrain(pods[i].type,      0, 6),
                    constrain(pods[i].steepness, 6, 48),
                    true, pods[i].on };
            }
        }
    }
    prefs.end();
}

/* ==================================================================
 *  /values JSON
 * ================================================================== */
static String buildValuesJSON() {
    static float    s_vol;
    static int      s_routing[NUM_OUTPUTS], s_delays[NUM_OUTPUTS];
    static float    s_gain[NUM_OUTPUTS];
    static bool     s_phase[NUM_OUTPUTS];
    static EQBand   s_eq[NUM_EQ_BANDS];
    static int      s_obCount[NUM_OUTPUTS];
    static OutBand  s_ob[NUM_OUTPUTS][MAX_OUT_BANDS];

    xSemaphoreTake(dspMutex, portMAX_DELAY);
    s_vol = volume;
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        s_routing[o] = routing[o];
        s_delays[o]  = delayInSamples[o];
        s_gain[o]    = outputGain[o];
        s_phase[o]   = outputPhase[o];
        s_obCount[o] = outBandCount[o];
        for (int i = 0; i < MAX_OUT_BANDS; i++) s_ob[o][i] = outBands[o][i];
    }
    for (int i = 0; i < NUM_EQ_BANDS; i++) s_eq[i] = sharedEQ[i];
    xSemaphoreGive(dspMutex);

    String j; j.reserve(2048);
    j += "{";
    j += "\"vol\":" + String(s_vol * 100.0f, 1) + ",";

    j += "\"routing\":[";
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        j += String(s_routing[o]); if (o < NUM_OUTPUTS - 1) j += ",";
    }
    j += "],\"delays\":[";
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        j += String(s_delays[o]); if (o < NUM_OUTPUTS - 1) j += ",";
    }
    j += "],\"outputGains\":[";
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        j += String((int)roundf(s_gain[o] * 100.0f));
        if (o < NUM_OUTPUTS - 1) j += ",";
    }
    j += "],\"outputPhase\":[";
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        j += s_phase[o] ? "true" : "false";
        if (o < NUM_OUTPUTS - 1) j += ",";
    }
    j += "],";

    j += "\"hw\":{\"sampleRate\":48000,\"switchFreq\":\"768k\",\"switchReg\":\"0x00\"},";

    j += "\"bands\":[";
    bool first = true;
    for (int i = 0; i < NUM_EQ_BANDS; i++) {
        if (!s_eq[i].active) continue;
        if (!first) j += ","; first = false;
        j += "{\"freq\":"      + String(s_eq[i].freq, 1)
           + ",\"gain\":"      + String(s_eq[i].gain, 2)
           + ",\"q\":"         + String(s_eq[i].q, 3)
           + ",\"type\":"      + String(s_eq[i].type)
           + ",\"steepness\":" + String(s_eq[i].steepness)
           + "}";
    }
    j += "],\"outputBands\":[";
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        j += "[";
        for (int i = 0; i < s_obCount[o]; i++) {
            if (i > 0) j += ",";
            j += "{\"freq\":"      + String(s_ob[o][i].freq, 1)
               + ",\"gain\":"      + String(s_ob[o][i].gain, 2)
               + ",\"q\":"         + String(s_ob[o][i].q, 3)
               + ",\"type\":"      + String(s_ob[o][i].type)
               + ",\"steepness\":" + String(s_ob[o][i].steepness)
               + ",\"on\":"        + (s_ob[o][i].on ? "true" : "false")
               + "}";
        }
        j += "]";
        if (o < NUM_OUTPUTS - 1) j += ",";
    }
    j += "]}";
    return j;
}

/* ==================================================================
 *  /set handler
 * ================================================================== */
static bool applyConfigJson(const char* buf, size_t total) {
    JsonDocument doc;
    DeserializationError err = deserializeJson(doc, buf, total);
    if (err) return false;

    float newVol = constrain((float)(doc["vol"] | (volume * 100.0f)), 0.0f, 100.0f) / 100.0f;

    int   newRouting[NUM_OUTPUTS];
    int   newDelay  [NUM_OUTPUTS];
    float newGain   [NUM_OUTPUTS];
    bool  newPhase  [NUM_OUTPUTS];
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        newRouting[o] = constrain((int)(doc["routing"][o] | routing[o]), -1, 1);
        newDelay[o]   = constrain((int)(doc["delays"][o]  | delayInSamples[o]),
                                  0, DELAY_MAX_SAMP - 1);
        float pct     = doc["outputGains"][o] | (outputGain[o] * 100.0f);
        newGain[o]    = constrain(pct / 100.0f, 0.0f, 1.0f);
        JsonVariantConst phaseVar = doc["outputPhase"][o];
        newPhase[o]   = phaseVar.isNull() ? (bool)outputPhase[o]
                                          : phaseVar.as<bool>();
    }

    EQBand newEQ[NUM_EQ_BANDS];
    for (int i = 0; i < NUM_EQ_BANDS; i++)
        newEQ[i] = {1000.0f, 0.0f, 0.7f, FTYPE_BYPASS, 12, false};
    JsonArray bands = doc["bands"].as<JsonArray>();
    int bi = 0;
    for (JsonObject b : bands) {
        if (bi >= NUM_EQ_BANDS) break;
        newEQ[bi].freq      = constrain((float)b["freq"],      20.0f, 20000.0f);
        newEQ[bi].gain      = constrain((float)b["gain"],     -18.0f, 18.0f);
        newEQ[bi].q         = constrain((float)b["q"],          0.1f, 10.0f);
        newEQ[bi].type      = constrain((int)b["type"],            0, 6);
        newEQ[bi].steepness = constrain((int)(b["steepness"] | 12), 6, 48);
        newEQ[bi].active    = true;
        bi++;
    }

    int     newOutCount[NUM_OUTPUTS] = {0};
    OutBand newOutBands[NUM_OUTPUTS][MAX_OUT_BANDS];
    for (int o = 0; o < NUM_OUTPUTS; o++)
        for (int i = 0; i < MAX_OUT_BANDS; i++)
            newOutBands[o][i] = {1000.0f, 0.0f, 0.7f, FTYPE_BYPASS, 12, false, true};

    JsonArray outputBands = doc["outputBands"].as<JsonArray>();
    int o = 0;
    for (JsonArray obs : outputBands) {
        if (o >= NUM_OUTPUTS) break;
        int cnt = 0;
        int nLpf = 0, nHpf = 0;   /* per-output crossover: <=1 High Cut, <=1 Low Cut */
        for (JsonObject b : obs) {
            if (cnt >= MAX_OUT_BANDS) break;
            int bt = constrain((int)b["type"], 0, 6);
            /* Per-output = crossover only: keep one High Cut + one Low Cut. */
            if      (bt == FTYPE_HIGH_CUT) { if (nLpf) continue; nLpf++; }
            else if (bt == FTYPE_LOW_CUT)  { if (nHpf) continue; nHpf++; }
            else continue;   /* drop peak / shelf / notch on outputs */
            newOutBands[o][cnt].freq      = constrain((float)b["freq"],      20.0f, 20000.0f);
            newOutBands[o][cnt].gain      = constrain((float)b["gain"],     -18.0f, 18.0f);
            newOutBands[o][cnt].q         = constrain((float)b["q"],          0.1f, 10.0f);
            newOutBands[o][cnt].type      = bt;
            newOutBands[o][cnt].steepness = constrain((int)(b["steepness"] | 12),  6, 48);
            JsonVariantConst onVar = b["on"];
            newOutBands[o][cnt].on        = onVar.isNull() ? true : onVar.as<bool>();
            newOutBands[o][cnt].active    = true;
            cnt++;
        }
        newOutCount[o++] = cnt;
    }

    /* Phase 1: publish the new SETTINGS into the globals. */
    xSemaphoreTake(dspMutex, portMAX_DELAY);
    for (int i = 0; i < NUM_EQ_BANDS; i++) sharedEQ[i] = newEQ[i];
    for (int oo = 0; oo < NUM_OUTPUTS; oo++) {
        outBandCount[oo] = newOutCount[oo];
        for (int i = 0; i < newOutCount[oo]; i++) outBands[oo][i] = newOutBands[oo][i];
    }
    xSemaphoreGive(dspMutex);

    /* Phase 2: heavy coefficient math into staging — NO lock held. */
    stageSharedEQ();
    stageOutBands();

    /* Phase 3: apply scalars + swap in staged coeffs atomically, then fade. */
    xSemaphoreTake(dspMutex, portMAX_DELAY);
    volume = newVol;
    g_volGain = powf(newVol, 2.5f);
    for (int oo = 0; oo < NUM_OUTPUTS; oo++) {
        routing[oo]     = newRouting[oo];
        outputGain[oo]  = newGain[oo];
        outputPhase[oo] = newPhase[oo];
        if (newDelay[oo] != delayInSamples[oo]) {
            memset(delayBuf[oo], 0, sizeof(delayBuf[oo]));
            delayHead[oo] = 0;
            delayInSamples[oo] = newDelay[oo];
        }
    }
    commitSharedEQ();
    commitOutBands();
    requestFade();
    xSemaphoreGive(dspMutex);

    requestSave();
    return true;
}

/* ==================================================================
 *  Setup
 * ================================================================== */
void setup() {
    Serial.setRxBufferSize(8192);   /* hold a whole preset between loop() drains */
    Serial.begin(115200);
    delay(300);

    statusLed.begin();
    statusLed.clear();
    statusLed.show();

    pinMode(PIN_BUTTON, INPUT_PULLUP);

    pinMode(PIN_ENC_A, INPUT_PULLUP);
    pinMode(PIN_ENC_B, INPUT_PULLUP);
    g_encState = (int8_t)((digitalRead(PIN_ENC_A) << 1) | digitalRead(PIN_ENC_B));
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_A), encoderISR, CHANGE);
    attachInterrupt(digitalPinToInterrupt(PIN_ENC_B), encoderISR, CHANGE);

    pinMode(PIN_LED_RED,   OUTPUT);
    pinMode(PIN_LED_GREEN, OUTPUT);
    digitalWrite(PIN_LED_RED,   LOW);
    digitalWrite(PIN_LED_GREEN, LOW);

    dspMutex = xSemaphoreCreateMutex();
    initSharedEQ();
    initOutBands();
    initDelayBuffers();
    loadSettings();
    stageSharedEQ();
    stageOutBands();
    xSemaphoreTake(dspMutex, portMAX_DELAY);
    commitSharedEQ();
    commitOutBands();
    xSemaphoreGive(dspMutex);

    if (amp_pdn_init_off() != ESP_OK) fatalSpin("PDN");
    if (i2c_bus_init()     != ESP_OK) fatalSpin("I2C");
    if (i2s_std_init()     != ESP_OK) fatalSpin("I2S");
    if (adc_init()         != ESP_OK) fatalSpin("ADC");
    if (amps_init() != ESP_OK) {
        Serial.println("[AMP] init had errors (continuing)");
        triggerError();
    }

    statusLedRefresh();


    /* Start the audio task first so the I2S clocks are running before we ask
     * the amps to lock onto them. */
    xTaskCreatePinnedToCore(audioTask, "audio", 16384, NULL, 24, NULL, 0);
    xTaskCreatePinnedToCore(saveTask,  "save",   8192, NULL,  1, &s_saveTask, 1);

    /* Short settle before releasing the amp output stages from HIZ into PLAY.
     * The I2S clocks have been streaming since i2s_std_init() (well before
     * this point), so 100 ms is only PSU-rail margin against a turn-on thump.
     * Raise it again if your supply ramps slowly and you hear a pop. */
    Serial.println("[AMP] settling before enabling outputs...");
    vTaskDelay(pdMS_TO_TICKS(100));
    amps_enable();
    Serial.println("[AMP] outputs enabled");
}

/* ==================================================================
 *  Serial "D" state dump (moved out of loop() so the console can call it)
 * ================================================================== */
static void dumpDspState() {
    float    s_volume;
    int      s_routing[NUM_OUTPUTS];
    int      s_delays[NUM_OUTPUTS];
    float    s_gains[NUM_OUTPUTS];
    bool     s_phase[NUM_OUTPUTS];
    int      s_activeShared;
    int      s_activeOut[NUM_OUTPUTS];
    EQBand   s_sharedEQ[NUM_EQ_BANDS];
    OutBand  s_outBands[NUM_OUTPUTS][MAX_OUT_BANDS];
    int      s_outBandCount[NUM_OUTPUTS];

    xSemaphoreTake(dspMutex, portMAX_DELAY);
    s_volume        = volume;
    s_activeShared  = activeSharedCount;
    for (int o = 0; o < NUM_OUTPUTS; o++) {
        s_routing[o]      = routing[o];
        s_delays[o]       = delayInSamples[o];
        s_gains[o]        = outputGain[o];
        s_phase[o]        = outputPhase[o];
        s_activeOut[o]    = activeOutCount[o];
        s_outBandCount[o] = outBandCount[o];
        for (int i = 0; i < MAX_OUT_BANDS; i++) s_outBands[o][i] = outBands[o][i];
    }
    for (int i = 0; i < NUM_EQ_BANDS; i++) s_sharedEQ[i] = sharedEQ[i];
    xSemaphoreGive(dspMutex);

    Serial.println("=== DSP STATE DUMP ===");
    Serial.printf("volume=%.3f  activeShared=%d  outBands=%d+%d+%d+%d\n",
                  s_volume, s_activeShared,
                  s_activeOut[0], s_activeOut[1], s_activeOut[2], s_activeOut[3]);
    Serial.printf("routing=%d %d %d %d\n",
                  s_routing[0], s_routing[1], s_routing[2], s_routing[3]);
    Serial.printf("delays=%d %d %d %d  gains=%.2f %.2f %.2f %.2f  phase=%d %d %d %d\n",
                  s_delays[0], s_delays[1], s_delays[2], s_delays[3],
                  s_gains[0],  s_gains[1],  s_gains[2],  s_gains[3],
                  s_phase[0],  s_phase[1],  s_phase[2],  s_phase[3]);
    for (int i = 0; i < NUM_EQ_BANDS; i++) {
        if (!s_sharedEQ[i].active) continue;
        Serial.printf("  sharedEQ[%d] t=%d f=%.1f g=%.2f q=%.3f s=%d\n",
                      i, s_sharedEQ[i].type, s_sharedEQ[i].freq,
                      s_sharedEQ[i].gain, s_sharedEQ[i].q, s_sharedEQ[i].steepness);
    }
    for (int oo = 0; oo < NUM_OUTPUTS; oo++) {
        Serial.printf("  out%d: %d bands\n", oo, s_outBandCount[oo]);
        for (int i = 0; i < s_outBandCount[oo]; i++) {
            Serial.printf("    [%d] t=%d f=%.1f g=%.2f q=%.3f on=%d\n",
                          i, s_outBands[oo][i].type, s_outBands[oo][i].freq,
                          s_outBands[oo][i].gain, s_outBands[oo][i].q, s_outBands[oo][i].on);
        }
    }
    Serial.println("=== END DUMP ===");
}

/* ==================================================================
 *  Serial console — line based.
 *    '{' ...  -> JSON command from the WebSerial UI (get / lv / set)
 *    single letter E/D/I -> legacy debug commands
 *  Non-JSON, non-single-letter lines are ignored. Debug prints from
 *  elsewhere are harmless: the host UI skips any line without a "t" tag.
 * ================================================================== */
static void serviceSerialConsole() {
    static String lineBuf;
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\r') continue;
        if (c != '\n') {
            if (lineBuf.length() < 16384) lineBuf += c;   /* guard runaway input */
            continue;
        }

        String line = lineBuf; lineBuf = "";
        line.trim();
        if (line.length() == 0) continue;

        if (line[0] == '{') {
            JsonDocument cmd;
            if (deserializeJson(cmd, line)) continue;     /* ignore malformed */
            const char* op = cmd["cmd"] | "";

            if (!strcmp(op, "get")) {
                /* One write so the line stays intact on the wire. */
                Serial.println(String("{\"t\":\"values\",\"d\":") + buildValuesJSON() + "}");

            } else if (!strcmp(op, "lv")) {
                const uint32_t now = millis();
                char buf[224];
                snprintf(buf, sizeof(buf),
                    "{\"t\":\"lv\",\"peaks\":[%.4f,%.4f,%.4f,%.4f],"
                    "\"clip\":[%s,%s,%s,%s],\"vol\":%.1f}",
                    g_meterPeak[0], g_meterPeak[1], g_meterPeak[2], g_meterPeak[3],
                    (now < g_meterClipUntilMs[0]) ? "true" : "false",
                    (now < g_meterClipUntilMs[1]) ? "true" : "false",
                    (now < g_meterClipUntilMs[2]) ? "true" : "false",
                    (now < g_meterClipUntilMs[3]) ? "true" : "false",
                    volume * 100.0f);
                Serial.println(buf);

            } else if (!strcmp(op, "set")) {
                /* The line itself is the preset (extra "cmd" key is ignored). */
                bool ok = applyConfigJson(line.c_str(), line.length());
                Serial.println(ok ? "{\"t\":\"ok\"}" : "{\"t\":\"err\"}");
            }
            continue;
        }

        if (line.length() == 1) {
            char cmd = line[0];
            if (cmd == 'E' || cmd == 'e') {
                Serial.println("Erasing NVS and rebooting...");
                nvs_flash_erase(); nvs_flash_init();
                delay(200); ESP.restart();
            } else if (cmd == 'D' || cmd == 'd') {
                dumpDspState();
            } else if (cmd == 'I' || cmd == 'i') {
                i2c_scan_print();
            }
        }
    }
}

/* ==================================================================
 *  Loop — runs at ~10 ms tick
 * ================================================================== */
void loop() {
    serviceSerialConsole();

    serviceDeferredSave();
    serviceControls();
    statusLedRefresh();
    amp_fault_service();

    vTaskDelay(pdMS_TO_TICKS(10));
}
