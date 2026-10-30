#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/stdio_usb.h"

#include "hardware/i2c.h"

// Split so that nothing can ever block the motors:
// - Core 1: inputs + motors only. Calls nothing but gpio_get/gpio_put and
//   time_us_64: no printf, no sleep, no I2C, no locks.
// - Core 0: USB console + VL53L0X sensors + ejection detection. May block
//   arbitrarily (missing sensors, no/hanging USB terminal) without core 1
//   noticing.

// --- Pinout according to the PCB (schematics.kicad_sch) ---
// Unit n is controlled by pin n of the screw terminals:
// Unit 1 = BTN0/TRG0 -> driver J2 (STEP0/DIR0) -> M1, sensor J3, etc.
#define NUM_UNITS 3
#define STP_EN_PIN 15 // shared by all drivers, active low, 10k pull-up (R1)

typedef struct {
    uint step_pin;
    uint dir_pin;
    uint btn_pin;   // to GND, internal pull-up
    uint trg_pin;   // active HIGH, internal pull-down (default LOW)
    uint xshut_pin;
    uint8_t vl53_addr; // target I2C address after bring-up
    const char *driver;   // driver socket
    const char *btn_term; // screw terminal pin
    const char *trg_term;
    const char *sensor_conn;
} unit_pins_t;

static const unit_pins_t UNIT[NUM_UNITS] = {
    { .step_pin = 10, .dir_pin = 9,  .btn_pin = 3, .trg_pin = 0, .xshut_pin = 19, .vl53_addr = 0x30,
      .driver = "J2", .btn_term = "J11.1", .trg_term = "J12.1", .sensor_conn = "J3" },
    { .step_pin = 12, .dir_pin = 11, .btn_pin = 4, .trg_pin = 1, .xshut_pin = 21, .vl53_addr = 0x31,
      .driver = "J6", .btn_term = "J11.3", .trg_term = "J12.2", .sensor_conn = "J8" },
    { .step_pin = 14, .dir_pin = 13, .btn_pin = 5, .trg_pin = 2, .xshut_pin = 27, .vl53_addr = 0x32,
      .driver = "J7", .btn_term = "J11.5", .trg_term = "J12.3", .sensor_conn = "J9" },
};

#define STEPS_PER_REV 3200
// Half step period. 714 us is tested on the PCB; 357 us (double speed)
// did not work.
#define STEP_HALF_PERIOD_US 714
// Upper limit for a dispense run in case the sensor never sees an ejection
// (e.g. dispenser empty).
#define DISPENSE_MAX_REVS 10
// 0: every start (click, long press, trigger) is exactly one revolution, the
// sensors are only read and displayed. 1: long press/trigger dispense until
// the sensor sees an ejection.
#define SENSOR_DISPENSE 0

// Minimum time a level has to be stable before it counts.
// Buttons bounce mechanically -> 20 ms. The trigger pulse has to be HIGH for
// at least TRG_MIN_PULSE_US, shorter pulses are ignored; falling back to LOW
// only needs a short glitch filter.
#define BTN_DEBOUNCE_US 20000
#define BTN_LONG_PRESS_US 2000000
#define TRG_MIN_PULSE_US 5000
#define TRG_FILTER_US 1000

// --- VL53L0X ---
#define I2C_PORT i2c0
#define SDA_PIN 16
#define SCL_PIN 17
#define I2C_BAUD 100000
#define I2C_TIMEOUT_US 10000
#define VL53_ADDR_DEFAULT 0x29
#define VL53_MODEL_ID 0xEE
// Number of consecutive failed readings after which a sensor is considered
// stuck and gets reinitialized on its own.
#define VL53_STUCK_TIMEOUTS 20
#define SENSOR_RETRY_INTERVAL_MS 5000

// Measurement loop period. 117 ms is the cadence of the original two-sensor
// loop (measured in the old session logs); FALL_MIN_DIP_SAMPLES is tuned to
// it, so it is kept fixed regardless of how many sensors are present.
#define SAMPLE_PERIOD_MS 117

// Graph scaling: one '#' per GRAPH_MM_PER_CHAR millimeters
#define GRAPH_MM_PER_CHAR 40
#define GRAPH_MAX_CHARS 30

// --- Unit state, shared between core 0 and core 1 ---
// INACTIVE -> REV:      short button click (core 1), exactly one revolution.
// INACTIVE -> DISPENSE: trigger pulse or long button press (core 1).
// DISPENSE ends when core 0 classifies an ejection, or - if the sensor is not
// available - after one revolution, or after DISPENSE_MAX_REVS revolutions.
typedef enum { UNIT_INACTIVE, UNIT_REV, UNIT_DISPENSE } unit_state_t;
typedef enum { SRC_BTN_CLICK, SRC_BTN_LONG, SRC_TRG } run_source_t;
typedef enum { END_REV_DONE, END_EJECTED, END_NO_SENSOR, END_MAX_REVS } run_end_t;

static volatile unit_state_t unit_state[NUM_UNITS];
static volatile uint32_t run_id[NUM_UNITS];         // core 1: incremented on every start
static volatile run_source_t run_source[NUM_UNITS];
static volatile uint32_t ejected_run_id[NUM_UNITS]; // core 0: run in which an ejection was classified
static volatile bool sensor_ok[NUM_UNITS];          // core 0: sensor initialized and delivering readings
// Core 1 -> core 0, for the console only
static volatile uint32_t ended_run_id[NUM_UNITS];
static volatile run_end_t end_reason[NUM_UNITS];
static volatile uint32_t end_steps[NUM_UNITS];

// --- Core 1: inputs + motors, time-driven instead of blocking, ---
// --- so that all units can run independently and simultaneously. ---
typedef struct {
    bool stable;   // debounced state (true = active)
    bool last_raw;
    uint64_t last_edge_us;
} input_t;

typedef enum { EDGE_NONE, EDGE_ACTIVATE, EDGE_RELEASE } edge_t;

typedef struct {
    input_t btn;
    input_t trg;
    uint64_t btn_press_us;
    bool btn_long_fired;
    uint32_t steps_done;
    bool step_pin_high;
    uint64_t next_toggle_us;
} motor_runtime_t;

// Returns the debounced edge as soon as the new state has been stable long
// enough (active: min_active_us, inactive: min_inactive_us).
static edge_t input_update(input_t *in, bool raw, uint64_t now_us,
                           uint32_t min_active_us, uint32_t min_inactive_us) {
    if (raw != in->last_raw) {
        in->last_raw = raw;
        in->last_edge_us = now_us;
    }
    uint32_t min_stable_us = raw ? min_active_us : min_inactive_us;
    if (raw != in->stable && (now_us - in->last_edge_us) >= min_stable_us) {
        in->stable = raw;
        return raw ? EDGE_ACTIVATE : EDGE_RELEASE;
    }
    return EDGE_NONE;
}

static void input_pin_init(uint pin, bool pull_up) {
    gpio_init(pin);
    gpio_set_dir(pin, GPIO_IN);
    if (pull_up) gpio_pull_up(pin);
    else gpio_pull_down(pin);
}

static void unit_start(int u, motor_runtime_t *r, run_source_t src, uint64_t now_us) {
    r->steps_done = 0;
    r->next_toggle_us = now_us;
    run_source[u] = src;
    run_id[u]++;
    unit_state[u] = (SENSOR_DISPENSE && src != SRC_BTN_CLICK) ? UNIT_DISPENSE : UNIT_REV;
}

static void unit_stop(int u, motor_runtime_t *r, run_end_t reason) {
    end_reason[u] = reason;
    end_steps[u] = r->steps_done;
    ended_run_id[u] = run_id[u];
    unit_state[u] = UNIT_INACTIVE;
}

static void core1_motor_task(void) {
    motor_runtime_t rt[NUM_UNITS] = {0};

    for (int u = 0; u < NUM_UNITS; u++) {
        gpio_init(UNIT[u].step_pin);
        gpio_init(UNIT[u].dir_pin);
        gpio_set_dir(UNIT[u].step_pin, GPIO_OUT);
        gpio_set_dir(UNIT[u].dir_pin, GPIO_OUT);
        gpio_put(UNIT[u].dir_pin, 1); // forward
        input_pin_init(UNIT[u].btn_pin, true);
        input_pin_init(UNIT[u].trg_pin, false);
    }

    gpio_init(STP_EN_PIN);
    gpio_set_dir(STP_EN_PIN, GPIO_OUT);
    gpio_put(STP_EN_PIN, 0); // enable drivers

    while (true) {
        uint64_t now_us = time_us_64();

        for (int u = 0; u < NUM_UNITS; u++) {
            motor_runtime_t *r = &rt[u];

            // Button: short click (HIGH -> LOW -> HIGH, released before 2 s)
            // = one revolution; held for 2 s = dispense, fired while still held.
            // Trigger: dispense after a full pulse LOW -> HIGH -> LOW, HIGH for
            // at least TRG_MIN_PULSE_US.
            // Always evaluate all inputs so that anything fired during a running
            // revolution is discarded instead of being replayed later.
            bool click = false, long_press = false;
            edge_t btn = input_update(&r->btn, !gpio_get(UNIT[u].btn_pin), now_us,
                                      BTN_DEBOUNCE_US, BTN_DEBOUNCE_US);
            if (btn == EDGE_ACTIVATE) {
                r->btn_press_us = now_us;
                r->btn_long_fired = false;
            } else if (btn == EDGE_RELEASE && !r->btn_long_fired) {
                click = true;
            } else if (r->btn.stable && !r->btn_long_fired && now_us - r->btn_press_us >= BTN_LONG_PRESS_US) {
                r->btn_long_fired = true;
                long_press = true;
            }
            bool trg = input_update(&r->trg, gpio_get(UNIT[u].trg_pin), now_us,
                                    TRG_MIN_PULSE_US, TRG_FILTER_US) == EDGE_RELEASE;

            if (unit_state[u] == UNIT_INACTIVE) {
                if (click) unit_start(u, r, SRC_BTN_CLICK, now_us);
                else if (long_press) unit_start(u, r, SRC_BTN_LONG, now_us);
                else if (trg) unit_start(u, r, SRC_TRG, now_us);
            }

            if (unit_state[u] == UNIT_DISPENSE && ejected_run_id[u] == run_id[u]) {
                unit_stop(u, r, END_EJECTED);
            }

            if (unit_state[u] != UNIT_INACTIVE && (int64_t)(now_us - r->next_toggle_us) >= 0) {
                r->step_pin_high = !r->step_pin_high;
                gpio_put(UNIT[u].step_pin, r->step_pin_high);
                r->next_toggle_us = now_us + STEP_HALF_PERIOD_US;
                if (!r->step_pin_high) { // falling edge = one step complete
                    r->steps_done++;
                    if (unit_state[u] == UNIT_REV) {
                        if (r->steps_done >= STEPS_PER_REV) unit_stop(u, r, END_REV_DONE);
                    } else if (!sensor_ok[u] && r->steps_done >= STEPS_PER_REV) {
                        // without a sensor there is no classification: at least one revolution
                        unit_stop(u, r, END_NO_SENSOR);
                    } else if (r->steps_done >= DISPENSE_MAX_REVS * STEPS_PER_REV) {
                        unit_stop(u, r, END_MAX_REVS);
                    }
                }
            }
        }
    }
}

// --- Core 0: VL53L0X driver ---
static void i2c_init_pins(void) {
    i2c_init(I2C_PORT, I2C_BAUD);
    gpio_set_function(SDA_PIN, GPIO_FUNC_I2C);
    gpio_set_function(SCL_PIN, GPIO_FUNC_I2C);
    gpio_pull_up(SDA_PIN);
    gpio_pull_up(SCL_PIN);
}

static bool reg_write(uint8_t addr, uint8_t reg, uint8_t val) {
    uint8_t buf[2] = {reg, val};
    return i2c_write_timeout_us(I2C_PORT, addr, buf, 2, false, I2C_TIMEOUT_US) == 2;
}

static bool reg_read(uint8_t addr, uint8_t reg, uint8_t *buf, int len) {
    if (i2c_write_timeout_us(I2C_PORT, addr, &reg, 1, true, I2C_TIMEOUT_US) != 1) return false;
    return i2c_read_timeout_us(I2C_PORT, addr, buf, len, false, I2C_TIMEOUT_US) == len;
}

static uint8_t reg_read8(uint8_t addr, uint8_t reg) {
    uint8_t v = 0;
    reg_read(addr, reg, &v, 1);
    return v;
}

static bool reg_write_multi(uint8_t addr, uint8_t reg, const uint8_t *src, int len) {
    uint8_t buf[7];
    buf[0] = reg;
    for (int i = 0; i < len; i++) buf[i + 1] = src[i];
    return i2c_write_timeout_us(I2C_PORT, addr, buf, len + 1, false, I2C_TIMEOUT_US) == len + 1;
}

static const uint8_t tuning_settings[][2] = {
    {0xFF,0x01},{0x00,0x00},{0xFF,0x00},{0x09,0x00},{0x10,0x00},{0x11,0x00},
    {0x24,0x01},{0x25,0xFF},{0x75,0x00},{0xFF,0x01},{0x4E,0x2C},{0x48,0x00},
    {0x30,0x20},{0xFF,0x00},{0x30,0x09},{0x54,0x00},{0x31,0x04},{0x32,0x03},
    {0x40,0x83},{0x46,0x25},{0x60,0x00},{0x27,0x00},{0x50,0x06},{0x51,0x00},
    {0x52,0x96},{0x56,0x08},{0x57,0x30},{0x61,0x00},{0x62,0x00},{0x64,0x00},
    {0x65,0x00},{0x66,0xA0},{0xFF,0x01},{0x22,0x32},{0x47,0x14},{0x49,0xFF},
    {0x4A,0x00},{0xFF,0x00},{0x7A,0x0A},{0x7B,0x00},{0x78,0x21},{0xFF,0x01},
    {0x23,0x34},{0x42,0x00},{0x44,0xFF},{0x45,0x26},{0x46,0x05},{0x40,0x40},
    {0x0E,0x06},{0x20,0x1A},{0x43,0x40},{0xFF,0x00},{0x34,0x03},{0x35,0x44},
    {0xFF,0x01},{0x31,0x04},{0x4B,0x09},{0x4C,0x05},{0x4D,0x04},{0xFF,0x00},
    {0x44,0x00},{0x45,0x20},{0x47,0x08},{0x48,0x28},{0x67,0x00},{0x70,0x04},
    {0x71,0x01},{0x72,0xFE},{0x76,0x00},{0x77,0x00},{0xFF,0x01},{0x0D,0x01},
    {0xFF,0x00},{0x80,0x01},{0x01,0xF8},{0xFF,0x01},{0x8E,0x01},{0x00,0x01},
    {0xFF,0x00},{0x80,0x00},
};

static bool wait_reg(uint8_t addr, uint8_t reg, uint8_t mask, bool until_set) {
    uint64_t t0 = time_us_64();
    while (((reg_read8(addr, reg) & mask) != 0) != until_set) {
        if (time_us_64() - t0 > 500000) return false;
    }
    return true;
}

static bool get_spad_info(uint8_t addr, uint8_t *count, bool *is_aperture) {
    reg_write(addr, 0x80, 0x01); reg_write(addr, 0xFF, 0x01); reg_write(addr, 0x00, 0x00);
    reg_write(addr, 0xFF, 0x06); reg_write(addr, 0x83, reg_read8(addr, 0x83) | 0x04);
    reg_write(addr, 0xFF, 0x07); reg_write(addr, 0x81, 0x01);
    reg_write(addr, 0x80, 0x01);
    reg_write(addr, 0x94, 0x6B); reg_write(addr, 0x83, 0x00);
    if (!wait_reg(addr, 0x83, 0xFF, true)) return false;
    reg_write(addr, 0x83, 0x01);
    uint8_t tmp = reg_read8(addr, 0x92);
    *count = tmp & 0x7F;
    *is_aperture = (tmp >> 7) & 0x01;
    reg_write(addr, 0x81, 0x00); reg_write(addr, 0xFF, 0x06);
    reg_write(addr, 0x83, reg_read8(addr, 0x83) & ~0x04);
    reg_write(addr, 0xFF, 0x01); reg_write(addr, 0x00, 0x01);
    reg_write(addr, 0xFF, 0x00); reg_write(addr, 0x80, 0x00);
    return true;
}

static bool ref_calibration(uint8_t addr, uint8_t vhv_init_byte) {
    reg_write(addr, 0x00, 0x01 | vhv_init_byte);
    if (!wait_reg(addr, 0x13, 0x07, true)) return false;
    reg_write(addr, 0x0B, 0x01);
    reg_write(addr, 0x00, 0x00);
    return true;
}

typedef struct {
    uint8_t addr;
    uint8_t stop_variable;
} vl53_t;

static bool vl53_init(vl53_t *s) {
    uint8_t addr = s->addr;
    uint8_t id = 0;
    if (!reg_read(addr, 0xC0, &id, 1) || id != VL53_MODEL_ID) return false;

    reg_write(addr, 0x89, reg_read8(addr, 0x89) | 0x01); // 2V8 I/O mode
    reg_write(addr, 0x88, 0x00);                         // I2C standard mode

    reg_write(addr, 0x80, 0x01);
    reg_write(addr, 0xFF, 0x01);
    reg_write(addr, 0x00, 0x00);
    s->stop_variable = reg_read8(addr, 0x91);
    reg_write(addr, 0x00, 0x01);
    reg_write(addr, 0xFF, 0x00);
    reg_write(addr, 0x80, 0x00);

    reg_write(addr, 0x60, reg_read8(addr, 0x60) | 0x12); // disable MSRC / pre-range limit checks
    uint8_t limit[2] = {0x00, 0x20};                     // final range signal rate limit 0.25 MCPS
    reg_write_multi(addr, 0x44, limit, 2);
    reg_write(addr, 0x01, 0xFF);

    uint8_t spad_count;
    bool spad_is_aperture;
    if (!get_spad_info(addr, &spad_count, &spad_is_aperture)) return false;

    uint8_t ref_spad_map[6];
    reg_read(addr, 0xB0, ref_spad_map, 6);

    reg_write(addr, 0xFF, 0x01);
    reg_write(addr, 0x4F, 0x00);
    reg_write(addr, 0x4E, 0x2C);
    reg_write(addr, 0xFF, 0x00);
    reg_write(addr, 0xB6, 0xB4);

    uint8_t first_spad = spad_is_aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad || enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x01) {
            enabled++;
        }
    }
    reg_write_multi(addr, 0xB0, ref_spad_map, 6);

    for (unsigned i = 0; i < sizeof(tuning_settings) / sizeof(tuning_settings[0]); i++)
        reg_write(addr, tuning_settings[i][0], tuning_settings[i][1]);

    reg_write(addr, 0x0A, 0x04);                         // interrupt: new sample ready
    reg_write(addr, 0x84, reg_read8(addr, 0x84) & ~0x10); // active low
    reg_write(addr, 0x0B, 0x01);

    reg_write(addr, 0x01, 0xE8);

    reg_write(addr, 0x01, 0x01);
    if (!ref_calibration(addr, 0x40)) return false; // VHV calibration
    reg_write(addr, 0x01, 0x02);
    if (!ref_calibration(addr, 0x00)) return false; // phase calibration
    reg_write(addr, 0x01, 0xE8);
    return true;
}

// Single-shot measurement. Returns distance in mm, or -1 on error/timeout.
static int vl53_read_mm(vl53_t *s) {
    uint8_t addr = s->addr;
    reg_write(addr, 0x80, 0x01);
    reg_write(addr, 0xFF, 0x01);
    reg_write(addr, 0x00, 0x00);
    reg_write(addr, 0x91, s->stop_variable);
    reg_write(addr, 0x00, 0x01);
    reg_write(addr, 0xFF, 0x00);
    reg_write(addr, 0x80, 0x00);

    reg_write(addr, 0x00, 0x01); // start single-shot ranging

    uint64_t t0 = time_us_64();
    while (reg_read8(addr, 0x00) & 0x01) {
        if (time_us_64() - t0 > 100000) return -1;
    }
    t0 = time_us_64();
    while ((reg_read8(addr, 0x13) & 0x07) == 0) {
        if (time_us_64() - t0 > 100000) return -1;
    }

    uint8_t buf[2];
    if (!reg_read(addr, 0x1E, buf, 2)) return -1;
    reg_write(addr, 0x0B, 0x01); // clear interrupt
    return (buf[0] << 8) | buf[1];
}

// Brings up a single sensor. All sensors wake up on the default address 0x29,
// so every sensor that is not (yet) working is held in XSHUT reset; working
// ones have already been moved to their own address. Hence 0x29 is free here,
// only this one sensor wakes up on it and is moved right away. Sensors can
// therefore be (re)initialized individually without disturbing the others.
static bool sensor_bringup(int u, vl53_t *s) {
    gpio_put(UNIT[u].xshut_pin, 0);
    sleep_ms(10);
    gpio_put(UNIT[u].xshut_pin, 1);
    sleep_ms(10);

    s->addr = VL53_ADDR_DEFAULT;
    uint8_t id = 0;
    if (vl53_init(s) &&
        reg_write(VL53_ADDR_DEFAULT, 0x8A, UNIT[u].vl53_addr & 0x7F) &&
        reg_read(UNIT[u].vl53_addr, 0xC0, &id, 1) && id == VL53_MODEL_ID) {
        s->addr = UNIT[u].vl53_addr;
        return true;
    }
    gpio_put(UNIT[u].xshut_pin, 0); // would block 0x29 for the others otherwise
    return false;
}

// --- Ejection detection via distance (runs on core 0), per unit ---
// Minimal approach: a dip only counts as a real falling part once it stays
// below the threshold for at least FALL_MIN_DIP_SAMPLES consecutive samples.
// Hypothesis (not yet fully confirmed): the coil only produces a short blip,
// a falling part stays in the field of view considerably longer.
#define FALL_ENTER_MM 110           // below this distance a "dip" starts
#define FALL_MIN_DIP_SAMPLES 6      // samples until a dip counts as an ejection
#define FALL_FAR_MM 9999            // substitute for "out of range" / timeout

static bool fall_detector_update(int *below_count, int eff_mm) {
    if (eff_mm < FALL_ENTER_MM) return ++(*below_count) >= FALL_MIN_DIP_SAMPLES;
    *below_count = 0;
    return false;
}

static const char *source_name(run_source_t src) {
    if (!SENSOR_DISPENSE) {
        switch (src) {
        case SRC_BTN_CLICK: return "button click, 1 revolution";
        case SRC_BTN_LONG:  return "button long press, 1 revolution";
        default:            return "trigger pulse, 1 revolution";
        }
    }
    switch (src) {
    case SRC_BTN_CLICK: return "button click, 1 revolution";
    case SRC_BTN_LONG:  return "button long press, dispense";
    default:            return "trigger pulse, dispense";
    }
}

static void print_mapping(void) {
    printf("\n=== Pinout ===\n");
    for (int u = 0; u < NUM_UNITS; u++) {
        const unit_pins_t *p = &UNIT[u];
        printf("Unit %d (M%d, driver %s): STEP=GP%u DIR=GP%u | BTN%d=GP%u (%s) | TRG%d=GP%u (%s)"
               " | sensor %s: XSHUT%d=GP%u, I2C 0x%02X\n",
               u + 1, u + 1, p->driver, p->step_pin, p->dir_pin,
               u, p->btn_pin, p->btn_term, u, p->trg_pin, p->trg_term,
               p->sensor_conn, u, p->xshut_pin, p->vl53_addr);
    }
    printf("STP_EN=GP%d (LOW = drivers enabled), I2C: SDA=GP%d SCL=GP%d\n", STP_EN_PIN, SDA_PIN, SCL_PIN);
    if (SENSOR_DISPENSE)
        printf("Button click = 1 revolution, button held %d s or trigger pulse = dispense until ejection\n\n",
               BTN_LONG_PRESS_US / 1000000);
    else
        printf("Button click, button held %d s or trigger pulse = 1 revolution (sensor dispense off)\n\n",
               BTN_LONG_PRESS_US / 1000000);
}

int main() {
    // Start the motors first of all, before USB/I2C
    multicore_launch_core1(core1_motor_task);

    stdio_init_all(); // does not wait for USB (PICO_STDIO_USB_CONNECT_WAIT_TIMEOUT_MS=0)
    i2c_init_pins();
    for (int u = 0; u < NUM_UNITS; u++) {
        gpio_init(UNIT[u].xshut_pin);
        gpio_set_dir(UNIT[u].xshut_pin, GPIO_OUT);
        gpio_put(UNIT[u].xshut_pin, 0); // hold all sensors in reset first
    }
    sleep_ms(10);

    vl53_t sensors[NUM_UNITS];
    int consecutive_timeouts[NUM_UNITS] = {0};
    int fall_below_count[NUM_UNITS] = {0};
    uint32_t detect_run[NUM_UNITS] = {0};
    uint32_t seen_run[NUM_UNITS] = {0};
    uint32_t seen_end[NUM_UNITS] = {0};
    bool reported_ok[NUM_UNITS] = {0};
    bool was_connected = false;
    uint32_t last_retry_ms = 0;
    bool first_retry = true;

    while (true) {
        absolute_time_t next_sample = make_timeout_time_ms(SAMPLE_PERIOD_MS);
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());

        bool connected = stdio_usb_connected();
        if (connected && !was_connected) {
            sleep_ms(100); // give the terminal a moment, otherwise the first line gets lost
            print_mapping();
            for (int u = 0; u < NUM_UNITS; u++) reported_ok[u] = !sensor_ok[u]; // report state again
        }
        was_connected = connected;

        // (Re)initialize missing sensors, so hot-plugged sensors are picked up too
        if (first_retry || now_ms - last_retry_ms >= SENSOR_RETRY_INTERVAL_MS) {
            first_retry = false;
            last_retry_ms = now_ms;
            for (int u = 0; u < NUM_UNITS; u++) {
                if (!sensor_ok[u] && sensor_bringup(u, &sensors[u])) {
                    consecutive_timeouts[u] = 0;
                    sensor_ok[u] = true;
                }
            }
        }

        int mm[NUM_UNITS];
        for (int u = 0; u < NUM_UNITS; u++) {
            mm[u] = -1;
            if (!sensor_ok[u]) continue;
            mm[u] = vl53_read_mm(&sensors[u]);
            if (mm[u] >= 0) {
                consecutive_timeouts[u] = 0;
            } else if (++consecutive_timeouts[u] >= VL53_STUCK_TIMEOUTS) {
                sensor_ok[u] = false; // dispense falls back to one revolution
                gpio_put(UNIT[u].xshut_pin, 0);
            }
        }

        for (int u = 0; u < NUM_UNITS; u++) {
            if (sensor_ok[u] != reported_ok[u]) {
                reported_ok[u] = sensor_ok[u];
                if (sensor_ok[u]) printf("U%d: sensor %s OK\n", u + 1, UNIT[u].sensor_conn);
                else printf("U%d: sensor %s not available\n", u + 1, UNIT[u].sensor_conn);
            }

            uint32_t id = run_id[u];
            if (id != seen_run[u]) {
                seen_run[u] = id;
                printf("U%d: start (%s)\n", u + 1, source_name(run_source[u]));
            }

            if (unit_state[u] == UNIT_DISPENSE && sensor_ok[u]) {
                if (id != detect_run[u]) { // new run: reset the detector
                    detect_run[u] = id;
                    fall_below_count[u] = 0;
                }
                int eff_mm = (mm[u] < 0 || mm[u] >= 8190) ? FALL_FAR_MM : mm[u];
                if (fall_detector_update(&fall_below_count[u], eff_mm)) ejected_run_id[u] = id;
            } else {
                fall_below_count[u] = 0;
            }

            uint32_t end = ended_run_id[u];
            if (end != seen_end[u]) {
                seen_end[u] = end;
                float revs = (float)end_steps[u] / STEPS_PER_REV;
                switch (end_reason[u]) {
                case END_REV_DONE:  printf("U%d: revolution done\n", u + 1); break;
                case END_EJECTED:   printf("U%d: ejection detected after %.2f revolutions\n", u + 1, revs); break;
                case END_NO_SENSOR: printf("U%d: no sensor, stopped after %.2f revolutions\n", u + 1, revs); break;
                case END_MAX_REVS:  printf("U%d: no ejection detected, stopped after %d revolutions\n",
                                           u + 1, DISPENSE_MAX_REVS); break;
                }
            }
        }

        char line[NUM_UNITS * (GRAPH_MAX_CHARS + 24) + 2];
        int pos = 0;
        for (int u = 0; u < NUM_UNITS; u++) {
            char bar[GRAPH_MAX_CHARS + 1];
            int n = 0;
            if (mm[u] >= 0 && mm[u] < 8190) {
                n = mm[u] / GRAPH_MM_PER_CHAR;
                if (n > GRAPH_MAX_CHARS) n = GRAPH_MAX_CHARS;
            }
            for (int i = 0; i < n; i++) bar[i] = '#';
            bar[n] = '\0';
            char st = unit_state[u] == UNIT_DISPENSE ? 'D' : unit_state[u] == UNIT_REV ? 'R' : ' ';
            if (sensor_ok[u]) pos += snprintf(line + pos, sizeof(line) - pos, "U%d%c %5d mm |%-*s ",
                                              u + 1, st, mm[u], GRAPH_MAX_CHARS, bar);
            else pos += snprintf(line + pos, sizeof(line) - pos, "U%d%c   --- mm |%-*s ",
                                 u + 1, st, GRAPH_MAX_CHARS, "");
        }
        printf("%s\n", line);

        sleep_until(next_sample);
    }
}
