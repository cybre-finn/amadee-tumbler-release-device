#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"

#include "hardware/i2c.h"

#define I2C_PORT i2c0
#define SDA_PIN 16  // physical pin 21
#define SCL_PIN 17  // physical pin 22
#define I2C_BAUD 100000
#define I2C_TIMEOUT_US 10000

// Zwei VL53L0X auf dem gleichen Bus: beide melden sich nach einem Reset auf
// der Default-Adresse 0x29. Ueber XSHUT wird immer nur ein Sensor gleichzeitig
// aus dem Reset genommen, direkt danach bekommt "alt" eine neue Adresse,
// damit "neu" anschliessend konfliktfrei auf 0x29 aufwachen kann.
#define VL53_ADDR_DEFAULT 0x29

// Graph scaling: one '#' per GRAPH_MM_PER_CHAR millimeters
#define GRAPH_MM_PER_CHAR 20
#define GRAPH_MAX_CHARS 60

// Anzahl aufeinanderfolgender Timeouts, nach der ein Sensor als haengend
// gilt und (zusammen mit dem anderen) per Reinit wiederbelebt wird.
#define VL53_STUCK_TIMEOUTS 20

// --- Zwei unabhaengige Dispenser-Einheiten (Treiber + Taster + Sensor) ---
// Beide teilen sich nur den MCU (Cores, I2C-Bus); Pins, State-Machine und
// Fall-Erkennung laufen pro Einheit komplett unabhaengig voneinander.
#define NUM_UNITS 2
#define UNIT_ALT 0
#define UNIT_NEU 1

typedef struct {
    uint step_pin;
    uint dir_pin;
    uint button_pin; // gegen GND, intern per Pull-Up
    uint xshut_pin;
    uint8_t vl53_addr; // Ziel-I2C-Adresse nach dem Bringup
} unit_pins_t;

static const unit_pins_t UNIT[NUM_UNITS] = {
    [UNIT_ALT] = { .step_pin = 2,  .dir_pin = 4,  .button_pin = 14, .xshut_pin = 18, .vl53_addr = 0x30 },
    [UNIT_NEU] = { .step_pin = 11, .dir_pin = 12, .button_pin = 10, .xshut_pin = 19, .vl53_addr = VL53_ADDR_DEFAULT },
};

#define DELAY_US 1000          // normale Geschwindigkeit (Referenz fuer den Speedup-Faktor)
#define HOLD_SPEEDUP 1.4f      // Faktor fuer die Dauerdrehung
#define HOLD_DELAY_US ((int)(DELAY_US / HOLD_SPEEDUP))
#define DEBOUNCE_MS 20
// Schritte fuer eine volle Umdrehung der Dispenser-Scheibe. War 3200
// (1/16-Mikroschritt), draaht damit aber real 2x statt 1x pro Tastendruck
// -> aktuelle Treiber-MS-Konfiguration entspricht offenbar 1/8, also halbiert.
// Bei Bedarf nochmal anpassen, falls es nicht exakt eine Umdrehung ist.
#define STEPS_PER_REV 1600

// --- Dispenser-State-Machine, je Einheit geteilt zwischen Core 0 und Core 1 ---
// INACTIVE -> RUNNING: Taster (Core 1).
// RUNNING -> INACTIVE: Fall-Erkennung ueber die Distanz (Core 0), oder eine
// volle Umdrehung ohne erkannten Fall (Core 1, Selbststopp).
typedef enum { DISP_INACTIVE, DISP_RUNNING } dispenser_state_t;
static volatile dispenser_state_t dispenser_state[NUM_UNITS] = { DISP_INACTIVE, DISP_INACTIVE };

// --- Core 1: beide Treiber + Taster, zeitgesteuert statt blockierend, damit ---
// --- beide Einheiten unabhaengig und gleichzeitig laufen koennen.          ---
typedef struct {
    bool stable_btn;    // entprellter Button-Zustand (true = gedrueckt)
    bool last_raw_btn;
    uint32_t last_edge_ms;
    uint32_t total_steps; // laeuft ueber alle Umdrehungen weiter (Winkel-Referenz)
    uint32_t step_target; // total_steps-Wert, bei dem die laufende Umdrehung endet
    bool step_pin_high;
    uint64_t next_toggle_us;
} motor_runtime_t;

static void core1_motor_task(void) {
    motor_runtime_t rt[NUM_UNITS] = {0};
    uint64_t now_us = time_us_64();

    for (int u = 0; u < NUM_UNITS; u++) {
        gpio_init(UNIT[u].step_pin);
        gpio_init(UNIT[u].dir_pin);
        gpio_set_dir(UNIT[u].step_pin, GPIO_OUT);
        gpio_set_dir(UNIT[u].dir_pin, GPIO_OUT);
        gpio_put(UNIT[u].dir_pin, 0); // vorwaerts

        gpio_init(UNIT[u].button_pin);
        gpio_set_dir(UNIT[u].button_pin, GPIO_IN);
        gpio_pull_up(UNIT[u].button_pin);

        rt[u].next_toggle_us = now_us;
    }

    while (true) {
        uint32_t now_ms = to_ms_since_boot(get_absolute_time());
        now_us = time_us_64();

        for (int u = 0; u < NUM_UNITS; u++) {
            motor_runtime_t *r = &rt[u];

            bool raw = !gpio_get(UNIT[u].button_pin); // LOW = gedrueckt (Pull-Up)
            if (raw != r->last_raw_btn) {
                r->last_raw_btn = raw;
                r->last_edge_ms = now_ms;
            }
            if (raw != r->stable_btn && (now_ms - r->last_edge_ms) > DEBOUNCE_MS) {
                r->stable_btn = raw;
                if (r->stable_btn && dispenser_state[u] == DISP_INACTIVE) { // entprellter Tastendruck
                    r->step_target = r->total_steps + STEPS_PER_REV;
                    dispenser_state[u] = DISP_RUNNING;
                }
            }

            if (dispenser_state[u] == DISP_RUNNING && (int64_t)(now_us - r->next_toggle_us) >= 0) {
                r->step_pin_high = !r->step_pin_high;
                gpio_put(UNIT[u].step_pin, r->step_pin_high);
                r->next_toggle_us = now_us + HOLD_DELAY_US;
                if (!r->step_pin_high) { // fallende Flanke = ein Schritt komplett
                    if (++r->total_steps >= r->step_target) {
                        dispenser_state[u] = DISP_INACTIVE; // Umdrehung komplett, kein Fall erkannt
                    }
                }
            }
        }
    }
}

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
    if (!reg_read(addr, 0xC0, &id, 1)) return false;
    printf("[0x%02X] Model ID: 0x%02X (expected 0xEE)\n", addr, id);

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
    if (!get_spad_info(addr, &spad_count, &spad_is_aperture)) {
        printf("[0x%02X] SPAD info timeout\n", addr);
        return false;
    }

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
    if (!ref_calibration(addr, 0x40)) { printf("[0x%02X] VHV calibration timeout\n", addr); return false; }
    reg_write(addr, 0x01, 0x02);
    if (!ref_calibration(addr, 0x00)) { printf("[0x%02X] phase calibration timeout\n", addr); return false; }
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

static void scan_bus(void) {
    printf("Kein VL53L0X antwortet. SDA=GPIO%d SCL=GPIO%d. ACKs:", SDA_PIN, SCL_PIN);
    for (int a = 1; a < 127; a++) {
        uint8_t dummy;
        if (i2c_read_timeout_us(I2C_PORT, a, &dummy, 1, false, I2C_TIMEOUT_US) >= 0) printf(" 0x%02X", a);
    }
    printf("\n");
}

// Nimmt immer nur einen Sensor gleichzeitig aus dem XSHUT-Reset. Jeder
// Sensor meldet sich danach kurz auf der Default-Adresse 0x29; bevor der
// naechste aufwacht, wird die aktuelle Einheit sofort auf ihre Zielsensor-
// adresse umgestellt (falls diese von 0x29 abweicht), damit es nie zwei
// Sensoren gleichzeitig auf derselben Adresse gibt.
static void vl53_dual_bringup(vl53_t sensors[NUM_UNITS]) {
    for (int u = 0; u < NUM_UNITS; u++) {
        gpio_init(UNIT[u].xshut_pin);
        gpio_set_dir(UNIT[u].xshut_pin, GPIO_OUT);
        gpio_put(UNIT[u].xshut_pin, 0); // beide Sensoren erst in Reset halten
    }
    sleep_ms(10);

    for (int u = 0; u < NUM_UNITS; u++) {
        gpio_put(UNIT[u].xshut_pin, 1); // genau diesen einen Sensor aufwecken
        sleep_ms(10);

        sensors[u].addr = VL53_ADDR_DEFAULT;
        while (!vl53_init(&sensors[u])) {
            scan_bus();
            sleep_ms(500);
        }

        if (UNIT[u].vl53_addr != VL53_ADDR_DEFAULT) {
            reg_write(VL53_ADDR_DEFAULT, 0x8A, UNIT[u].vl53_addr & 0x7F);
            sensors[u].addr = UNIT[u].vl53_addr;
        }
    }
}

// --- Fall-Erkennung ueber die Distanz (laeuft auf Core 0), je Einheit ---
// Minimal-Ansatz: ein Dip gilt erst als echtes fallendes Teil, wenn er
// mindestens FALL_MIN_DIP_SAMPLES Samples lang ununterbrochen unter der
// Schwelle bleibt. Hypothese (noch nicht final bestaetigt): die Spule
// erzeugt nur einen kurzen Blip, ein durchfallendes Teil bleibt deutlich
// laenger im Sichtfeld.
#define FALL_ENTER_MM 110           // unterhalb dieser Distanz beginnt ein "Dip"
#define FALL_MIN_DIP_SAMPLES 6      // ~300ms bei 50ms Schleife, bis ein Dip als Fall zaehlt
#define FALL_FAR_MM 9999            // Ersatzwert fuer "out of range" / Timeout

typedef enum { FALL_IDLE, FALL_DIP } fall_state_t;

static void fall_detector_update(int unit, fall_state_t *state, int *below_count, int eff_mm) {
    if (eff_mm < FALL_ENTER_MM) {
        *state = FALL_DIP;
        if (++(*below_count) >= FALL_MIN_DIP_SAMPLES && dispenser_state[unit] == DISP_RUNNING) {
            dispenser_state[unit] = DISP_INACTIVE;
        }
    } else {
        *state = FALL_IDLE;
        *below_count = 0;
    }
}

int main() {
    stdio_init_all();
    i2c_init_pins();

    // Core 1 (Taster + Motoren, beide Einheiten) sofort starten, unabhaengig
    // von einer USB-Verbindung
    multicore_launch_core1(core1_motor_task);

    printf("VL53L0X dual live distance test\n");

    vl53_t sensors[NUM_UNITS];
    vl53_dual_bringup(sensors);

    int consecutive_timeouts[NUM_UNITS] = {0};
    fall_state_t fall_state[NUM_UNITS] = { FALL_IDLE, FALL_IDLE };
    int fall_below_count[NUM_UNITS] = {0};

    while (true) {
        int mm[NUM_UNITS];
        bool need_reinit = false;

        for (int u = 0; u < NUM_UNITS; u++) {
            mm[u] = vl53_read_mm(&sensors[u]);
            if (mm[u] < 0) {
                if (++consecutive_timeouts[u] >= VL53_STUCK_TIMEOUTS) need_reinit = true;
            } else {
                consecutive_timeouts[u] = 0;
            }
        }

        if (need_reinit) {
            printf("sensor stuck, reinitializing both units\n");
            i2c_init_pins();
            vl53_dual_bringup(sensors);
            for (int u = 0; u < NUM_UNITS; u++) {
                consecutive_timeouts[u] = 0;
                fall_state[u] = FALL_IDLE;
                fall_below_count[u] = 0;
            }
            continue;
        }

        char bars[NUM_UNITS][GRAPH_MAX_CHARS + 1];
        for (int u = 0; u < NUM_UNITS; u++) {
            if (dispenser_state[u] == DISP_RUNNING) {
                int eff_mm = (mm[u] < 0 || mm[u] >= 8190) ? FALL_FAR_MM : mm[u];
                fall_detector_update(u, &fall_state[u], &fall_below_count[u], eff_mm);
            } else {
                fall_state[u] = FALL_IDLE;
                fall_below_count[u] = 0;
            }

            int n = 0;
            if (mm[u] >= 0 && mm[u] < 8190) {
                n = mm[u] / GRAPH_MM_PER_CHAR;
                if (n > GRAPH_MAX_CHARS) n = GRAPH_MAX_CHARS;
            }
            for (int i = 0; i < n; i++) bars[u][i] = '#';
            bars[u][n] = '\0';
        }

        printf("U1 %5d mm |%-60s U2 %5d mm |%-60s\n", mm[UNIT_ALT], bars[UNIT_ALT], mm[UNIT_NEU], bars[UNIT_NEU]);
        sleep_ms(50);
    }
}
