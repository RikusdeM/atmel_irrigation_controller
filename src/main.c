#include <string.h>
#include <ctype.h>
#include <stdbool.h>
#include <asf.h>
#include <stdio.h>

#include "uart.h"
#include "crc.h"
#include "i2c.h"
#include "rfid.h"
#include "awt.h"
#include "owire.h"
#include "ds18b20.h"
#include "epoch.h"
#include "ram_hex.h"

#define TEMP_SENSOR_TANK                                0
#define TEMP_SENSOR_OUTLET                              1
#define TEMP_SENSOR_INLET                               2
#define ELEC_PULSE_COUNTER                              0
#define FLOW_PULSE_COUNTER                              1
#define N_PULSE_COUNTERS                                2
#define MODEL_TEMP_SCALE_FACTOR                         16


#define DS2483_ADDR                                     0b0011000

#define ASSET_ID                                        "GML-001"
#define CONFIG_PATH                                     ASSET_ID ".config"
#define STATUS_PATH                                     ASSET_ID ".status"

#define STATUS_STATE_TAG                                "s"
#define STATUS_FIRMWARE_TAG                             "fwv"
#define STATUS_RESET_REASON_TAG                         "rr"
#define STATUS_EPOCH_TAG                                "utc"
#define STATUS_TEMP1_TAG                                "t1_x16"
#define STATUS_TEMP2_TAG                                "t2_x16"
#define STATUS_TEMP3_TAG                                "t3_x16"
#define STATUS_PULSE1_TOTAL_TAG                         "pc1t"
#define STATUS_PULSE1_RESET_TAG                         "pc1r"
#define STATUS_PULSE2_TOTAL_TAG                         "pc2t"
#define STATUS_PULSE2_RESET_TAG                         "pc2r"
#define STATUS_ELEMENT_ON_TAG                           "eo"
#define STATUS_POWER_ON_TAG                             "po"
#define STATUS_WATER_RUNNING_TAG                        "wr"
#define STATUS_TEMP1_CODE_TAG                           "st1c"
#define STATUS_TEMP2_CODE_TAG                           "st2c"
#define STATUS_TEMP3_CODE_TAG                           "st3c"
#define CONFIG_REPORT_HOME_PERIOD_TAG                   "rhp"
#define CONFIG_TEMP1_CODE_TAG                           "t1c"
#define CONFIG_TEMP2_CODE_TAG                           "t2c"
#define CONFIG_TEMP3_CODE_TAG                           "t3c"
#define CONFIG_TEMP_DELTA_TRIGGER_TAG                   "tdt_x16"
#define CONFIG_ELEC_COUNTER_DELTA_TRIGGER_TAG           "p1dt"
#define CONFIG_FLOW_COUNTER_DELTA_TRIGGER_TAG           "p2dt"

#define CONFIG_CONTROL_POOL_TIMER_1_WEEKDAY_TAG         "pt1"
#define CONFIG_CONTROL_POOL_TIMER_2_WEEKDAY_TAG         "pt2"
#define CONFIG_CONTROL_POOL_TIMER_3_WEEKDAY_TAG         "pt3"

#define CONFIG_CONTROL_POOL_TIMER_1_WEEKEND_TAG         "pt_b1"
#define CONFIG_CONTROL_POOL_TIMER_2_WEEKEND_TAG         "pt_b2"
#define CONFIG_CONTROL_POOL_TIMER_3_WEEKEND_TAG         "pt_b3"

#define CONFIG_WATER_RUNNING_POOL_TIMER_1_WEEKDAY_TAG   "pt_c1"
#define CONFIG_WATER_RUNNING_POOL_TIMER_2_WEEKDAY_TAG   "pt_c2"
#define CONFIG_WATER_RUNNING_POOL_TIMER_3_WEEKDAY_TAG   "pt_c3"

#define CONFIG_WATER_RUNNING_POOL_TIMER_1_WEEKEND_TAG   "pt_d1"
#define CONFIG_WATER_RUNNING_POOL_TIMER_2_WEEKEND_TAG   "pt_d2"
#define CONFIG_WATER_RUNNING_POOL_TIMER_3_WEEKEND_TAG   "pt_d3"

#define CONFIG_CONTROL_TEMP_SP_TAG                      "tsp_x16"
#define CONFIG_CONTROL_SELECT_TAG                       "cms"
#define CONFIG_ENABLE_EVENT_HI_TAG                      "ee_hi"
#define CONFIG_ENABLE_EVENT_LO_TAG                      "ee_lo"
#define CONFIG_TEMP1_HI_TRIGGER_TAG                     "t1th_x16"
#define CONFIG_TEMP1_LO_TRIGGER_TAG                     "t1tl_x16"

#define EVENT_TEMP1_HI                                  100
#define EVENT_TEMP1_LO                                  101
#define EVENT_NO_TEMP_SENSORS                           110
#define EVENT_NEW_TEMP1_SENSOR                          111
#define EVENT_NEW_TEMP2_SENSOR                          112
#define EVENT_NEW_TEMP3_SENSOR                          113
#define EVENT_WATER_RUNNING                             120
#define EVENT_ELECTRICAL_SHORT                          130
#define EVENT_CRITICAL                                  200
#define EVENT_ERROR                                     201
#define EVENT_FATAL_ERROR                               202
#define EVENT_INFO                                      203
#define EVENT_WARNING                                   204

#define STATUS_FIRMWARE_VERSION_STR                     "GML-001-1.2.7"
#define STATUS_STATE_OPERATIONAL                        1
#define STATUS_STATE_SENSOR_ERROR                       2

#define STATUS_BUFFER_SIZE                              16
#define EVENT_BUFFER_SIZE                               16

#define CONFIG_EEPROM_VERSION                           0x03
#define CONFIG_DEFAULT_REPORT_HOME_PERIOD               (10 * 60)

#define CONFIG_DEFAULT_TEMP1_HI_TRIGGER_X16             (65 * 16)
#define CONFIG_DEFAULT_TEMP1_LO_TRIGGER_X16             (10 * 16)

#define CONFIG_DEFAULT_TEMP_DELTA_TRIG_X16              (1 * 16)
#define CONFIG_DEFAULT_TEMP_SP_X16                      (60 * 16)


#define TEMP_TRIGGER_RESET_X16                          (1 * 16)

#define FLOW_PULSE_PER_LITRE                            2
#define FLOW_TRIGGER_LITRE                              20
#define CONFIG_DEFAULT_FLOW_DELTA_TRIGGER               (FLOW_PULSE_PER_LITRE * FLOW_TRIGGER_LITRE)

#define ELEC_PULSE_PER_KWH                              1600
#define ELEC_GEYSER_KWH                                 3
#define ELEC_GEYSER_TRIGGER_MIN                         3
#define CONFIG_DEFAULT_ELEC_DELTA_TRIGGER               (((ELEC_GEYSER_KWH * ELEC_PULSE_PER_KWH) * ELEC_GEYSER_TRIGGER_MIN) / 60)

#define SECS_NETWORK_INACTIVE_TRIGGER                   (config.report_home_period_secs * 3)

#define DUMMY_WRITE                                     "at+awtda=a,\"ICA1\",1021099"
#define DUMMY                                           "at+awtda=a,\"ICA1\","
#define RTC_TIME                                        "at+rtc?"

char temp_string [40];


typedef struct _asset_model {
    char job_type;
    int32_t job_id;
    char* asset_id;
    char* job_desc;
    char* data_type;
    int32_t job_value;
    char* crc;
} asset_model;


typedef struct _config_0x02_t {
    uint8_t eeprom_version;
    uint16_t report_home_period_secs;
    struct _old_rom_t {
        bool valid;
        uint8_t code[8];
    } old_rom[3];
    int16_t temp_delta_trigger_x16;
    int32_t flow_delta_trigger;
    int32_t elec_delta_trigger;
    uint32_t pTimer[3];
    int16_t temp_sp_x16;
    enum {OLD_CONTROL_SELECT_OFF, OLD_CONTROL_SELECT_BYPASS, OLD_CONTROL_SELECT_POOL_TIMER} ctrl_select;
    bool enable_temp1_hi_event;
    bool enable_temp1_lo_event;
    int16_t temp1_hi_trigger_x16;
    int16_t temp1_lo_trigger_x16;
    uint8_t crc8;
} config_0x02_t;

typedef struct _eeprom_pulse_counters_0x02_t {
    uint32_t counter[N_PULSE_COUNTERS];
    uint32_t midnight_counter[N_PULSE_COUNTERS];
    uint8_t crc;
} eeprom_pulse_counters_0x02_t;

typedef struct _config_0x03_t {
    uint8_t eeprom_version;

    uint16_t report_home_period_secs;
    struct _rom_t {
        bool valid;
        uint8_t code[8];
    } rom[3];
    int16_t temp_delta_trigger_x16;
    int32_t flow_delta_trigger;
    int32_t elec_delta_trigger;
    uint32_t pTimerWeekDayControl[3];
    int16_t temp_sp_x16;
    enum {CONTROL_SELECT_OFF, CONTROL_SELECT_BYPASS, CONTROL_SELECT_POOL_TIMER} ctrl_select;
    bool enable_temp1_hi_event;
    bool enable_temp1_lo_event;
    int16_t temp1_hi_trigger_x16;
    int16_t temp1_lo_trigger_x16;
    uint32_t pTimerWeekEndControl[3];
    uint32_t pTimerWeekDayWaterRunning[3];
    uint32_t pTimerWeekEndWaterRunning[3];
    uint8_t crc8;
} config_0x03_t;

typedef struct _eeprom_pulse_counters_0x03_t {
    uint32_t counter[N_PULSE_COUNTERS];
    uint32_t midnight_counter[N_PULSE_COUNTERS];
    uint8_t crc;
} eeprom_pulse_counters_0x03_t;

typedef struct _eeprom_0x02_t {
    uint8_t buffer_0x02[((sizeof(config_0x02_t) + sizeof(eeprom_pulse_counters_0x02_t) + 31) / 32) * 32];
    config_0x02_t config_0x02;
    eeprom_pulse_counters_0x02_t pulse_counters_0x02;
} eeprom_0x02_t;

typedef struct _eeprom_0x03_t {
    uint8_t buffer_0x02[((sizeof(config_0x02_t) + sizeof(eeprom_pulse_counters_0x02_t) + 31) / 32) * 32];
    config_0x03_t config;
    eeprom_pulse_counters_0x03_t pulse_counters;
} eeprom_0x03_t;

typedef struct _status_t {
    uint32_t epoch;
    int8_t state;
    enum {SIGNALLED_NONE, SIGNALLED_OK, SIGNALLED_HI, SIGNALLED_LO} temp1_signalled;
    int16_t temp_x16[3];
    uint32_t pulse_counter[N_PULSE_COUNTERS];
    bool power_on;
    bool element_on;
    struct _water_event_t {
        bool running;
        bool sent;
    } water_event;
    bool electrical_short_event_sent;
    uint32_t pulse_counter_power_off;
} status_t;

typedef struct _event_t {
    uint32_t timestamp;
    uint8_t event;
    bool is_int;
    union {
        int16_t int_data;
        const char* str_data;
    };
} event_t;

typedef struct _event_buffer_t {
    event_t buffer[EVENT_BUFFER_SIZE];
    uint8_t front;
    uint8_t back;
} event_buffer_t;

typedef struct _status_buffer_t {
    status_t buffer[STATUS_BUFFER_SIZE];
    uint8_t front;
    uint8_t back;
} status_buffer_t;

typedef struct _modem_t {
    bool up;
    bool busy;
    bool send_id;
    bool send_config1;
    bool send_config2;
    bool send_rom_config;
    uint16_t secs_network_inactive;
    uint16_t secs_since_last_report_home;
} modem_t;

typedef struct _geyser_t {
    struct _water_t {
        uint32_t detected_at_epoch;
        bool detected;
    } water_running;
    struct _elect_t {
        uint32_t detected_at_epoch;  // better name than triggered required
        bool detected;
    } elect;
} geyser_t;

typedef struct _state_t {
    reset_cause_t reset_reason;
    modem_t modem;
    event_buffer_t event_buffer;
    status_buffer_t status_buffer;
    status_t current;
    status_t dispatched;
    uint32_t midnight_pulse_counter[N_PULSE_COUNTERS];
    geyser_t geyser;
    struct _debounce_t {
        bool level;
        uint8_t debounce;
    } count1, count2;
} state_t;


static config_0x03_t config;
static state_t state;
static twi_master_options_t opt;

static void buffer_current_status(void);
static void buffer_event(uint8_t event, const int32_t data);
static void buffer_str_event(uint8_t event, const char* data);
static int8_t config_find_ow_rom_code(uint8_t cfg_idx);


/*
* ****************************************************************************
*      ASSET STATE
* ****************************************************************************
*/

static uint32_t get_current_pulse_counter(uint8_t i)
{
    uint32_t v;
    irqflags_t f = cpu_irq_save();
    v = state.current.pulse_counter[i];
    cpu_irq_restore(f);
    return v;
}

static uint32_t get_midnight_pulse_counter(uint8_t i)
{
    uint32_t v;
    irqflags_t f = cpu_irq_save();
    v = state.midnight_pulse_counter[i];
    cpu_irq_restore(f);
    return v;
}

static void set_current_pulse_counter(uint8_t i, uint32_t v)
{
    irqflags_t f = cpu_irq_save();
    state.current.pulse_counter[i] = v;
    cpu_irq_restore(f);
}

static void set_midnight_pulse_counter(uint8_t i, uint32_t v)
{
    irqflags_t f = cpu_irq_save();
    state.midnight_pulse_counter[i] = v;
    cpu_irq_restore(f);
}

static void geyser_set_power_on(void)
{
    ioport_set_pin_high(GPIO_POWER);
}

static void geyser_set_power_off(void)
{
    if (ioport_get_value(GPIO_POWER)) {
        state.current.pulse_counter_power_off = state.current.pulse_counter[ELEC_PULSE_COUNTER];
    }
    ioport_set_pin_low(GPIO_POWER);
}

static bool geyser_is_power_on(void)
{
    return ioport_get_value(GPIO_POWER);
}

static bool geyser_is_element_on(void)
{
    if (state.geyser.elect.detected) {
        uint32_t epoch;
        irqflags_t f = cpu_irq_save();
        epoch = state.geyser.elect.detected_at_epoch;
        cpu_irq_restore(f);
        if (epoch_get() >= epoch + 10) {
            state.geyser.elect.detected = false;
        }
    }
    return state.geyser.elect.detected;
}

static bool geyser_is_water_running(void)
{
    if (state.geyser.water_running.detected) {
        uint32_t epoch;
        irqflags_t f = cpu_irq_save();
        epoch = state.geyser.water_running.detected_at_epoch;
        cpu_irq_restore(f);
        if (epoch_get() >= epoch + 10) {
            state.geyser.water_running.detected = false;
        }
    }
    return state.geyser.water_running.detected;
}

static bool is_weekday(uint32_t epoch)
{
    uint32_t day = (((epoch + EPOCH_LOCAL_OFFSET) / (24UL * 60 * 60)) + 3) % 7;
    return day < 5;
}

static bool water_running_pool_timer_on(void)
{
    uint32_t epoch = epoch_get();
    uint32_t sec_of_day = (epoch + EPOCH_LOCAL_OFFSET) % (24UL * 60 * 60);
    uint8_t timeslot_15min = sec_of_day / (15 * 60);
    uint8_t timer = timeslot_15min / 32;
    uint8_t bit = timeslot_15min % 32;
    uint32_t mask = 1UL << bit;
    uint32_t val = is_weekday(epoch) ? config.pTimerWeekDayWaterRunning[timer] : config.pTimerWeekEndWaterRunning[timer];
    return (val & mask) > 0;
}

static bool control_pool_timer_on(void)
{
    uint32_t epoch = epoch_get();
    uint32_t sec_of_day = (epoch + EPOCH_LOCAL_OFFSET) % (24UL * 60 * 60);
    uint8_t timeslot_15min = sec_of_day / (15 * 60);
    uint8_t timer = timeslot_15min / 32;
    uint8_t bit = timeslot_15min % 32;
    uint32_t mask = 1UL << bit;
    uint32_t val = is_weekday(epoch) ? config.pTimerWeekDayControl[timer] : config.pTimerWeekEndControl[timer];
    return (val & mask) > 0;
}

static void update_current_status(void)
{
    int8_t index[3] = {
        config_find_ow_rom_code(0),
        config_find_ow_rom_code(1),
        config_find_ow_rom_code(2)
    };
    state.current.epoch = epoch_get();
    state.current.power_on = geyser_is_power_on();
    state.current.element_on = geyser_is_element_on();
    state.current.water_event.running = geyser_is_water_running();
    state.current.temp_x16[0] = index[0] >= 0
                                ? ds18b20_temperature[index[0]]
                                : INT16_MIN;
    state.current.temp_x16[1] = index[1] >= 0
                                ? ds18b20_temperature[index[1]]
                                : INT16_MIN;
    state.current.temp_x16[2] = index[2] >= 0
                                ? ds18b20_temperature[index[2]]
                                : INT16_MIN;
    if (state.current.temp_x16[0] == INT16_MIN || state.current.temp_x16[1] == INT16_MIN
            || state.current.temp_x16[2] == INT16_MIN) {
        state.current.state = STATUS_STATE_SENSOR_ERROR;
    } else {
        state.current.state = STATUS_STATE_OPERATIONAL;
    }
}

static bool status_change_detected(void)
{
    return
        (
            (state.modem.secs_since_last_report_home >= config.report_home_period_secs)

            || (state.dispatched.power_on != state.current.power_on)

            || (state.dispatched.element_on != state.current.element_on)

            || (state.dispatched.water_event.running != state.current.water_event.running)

            || (state.current.temp_x16[0] > INT16_MIN
                && labs(state.dispatched.temp_x16[0] - state.current.temp_x16[0]) >= config.temp_delta_trigger_x16)

            || (state.current.temp_x16[1] > INT16_MIN
                && labs(state.dispatched.temp_x16[1] - state.current.temp_x16[1]) >= config.temp_delta_trigger_x16)

            || (state.current.temp_x16[2] > INT16_MIN
                && labs(state.dispatched.temp_x16[2] - state.current.temp_x16[2]) >= config.temp_delta_trigger_x16)

            || ((get_current_pulse_counter(ELEC_PULSE_COUNTER) -
                 state.dispatched.pulse_counter[ELEC_PULSE_COUNTER]) >= config.elec_delta_trigger)

            || ((get_current_pulse_counter(FLOW_PULSE_COUNTER) -
                 state.dispatched.pulse_counter[FLOW_PULSE_COUNTER]) >= config.flow_delta_trigger)
        );
}

static void check_temp1_event(uint8_t event_hi, uint8_t event_lo)
{
    if (state.current.temp_x16[0] > INT16_MIN) {
        if (state.current.temp_x16[0] >= config.temp1_hi_trigger_x16 && state.current.temp1_signalled != SIGNALLED_HI) {
            if (config.enable_temp1_hi_event) {
                buffer_event(event_hi, state.current.temp_x16[0]);
            }
            state.current.temp1_signalled = SIGNALLED_HI;
        } else if (state.current.temp_x16[0] <= config.temp1_lo_trigger_x16 && state.current.temp1_signalled != SIGNALLED_LO) {
            if (config.enable_temp1_lo_event) {
                buffer_event(event_lo, state.current.temp_x16[0]);
            }
            state.current.temp1_signalled = SIGNALLED_LO;
        } else {
            if (state.current.temp1_signalled != SIGNALLED_OK) {
                if (state.current.temp_x16[0] > config.temp1_lo_trigger_x16 + TEMP_TRIGGER_RESET_X16 &&
                        state.current.temp_x16[0] < config.temp1_hi_trigger_x16 - TEMP_TRIGGER_RESET_X16) {
                    state.current.temp1_signalled = SIGNALLED_OK;
                }
            }
        }
    }
}

static void check_water_flow_event(void)
{
    if (water_running_pool_timer_on()) {
        if (state.current.water_event.running && !state.current.water_event.sent) {
            state.current.water_event.sent = true;
            buffer_str_event(EVENT_WATER_RUNNING, NULL);
        }
    } else {
        state.current.water_event.sent = false;
    }
}

static void check_electrical_short_event(void)
{
    if (ioport_get_value(GPIO_POWER)) {
        state.current.electrical_short_event_sent = false;
    } else {
        if ((state.current.pulse_counter_power_off != state.current.pulse_counter[ELEC_PULSE_COUNTER])
                && !state.current.electrical_short_event_sent) {
            buffer_str_event(EVENT_ELECTRICAL_SHORT, NULL);
            state.current.electrical_short_event_sent = true;
        }
    }
}

static void update_asset_state(void)
{
    update_current_status();
    if (status_change_detected()) {
        buffer_current_status();
    }

    check_temp1_event(EVENT_TEMP1_HI, EVENT_TEMP1_LO);
    check_water_flow_event();
    check_electrical_short_event();

    switch (config.ctrl_select) {
    case CONTROL_SELECT_OFF:
        geyser_set_power_off();
        break;
    case CONTROL_SELECT_BYPASS:
        if (state.current.temp_x16[TEMP_SENSOR_TANK] < config.temp1_hi_trigger_x16) {
            geyser_set_power_on();
        } else {
            geyser_set_power_off();
        }
        break;
    case CONTROL_SELECT_POOL_TIMER:
        if (control_pool_timer_on()) {
            if (state.current.temp_x16[TEMP_SENSOR_TANK] <= (config.temp_sp_x16 - config.temp_delta_trigger_x16)) {
                if (state.current.temp_x16[TEMP_SENSOR_TANK] > INT16_MIN) {
                    geyser_set_power_on();
                } else {
                    geyser_set_power_off();
                }
            } else if (state.current.temp_x16[TEMP_SENSOR_TANK] >= config.temp_sp_x16) {
                geyser_set_power_off();
            }
        } else {
            geyser_set_power_off();
        }
        break;
    }
}


/*
* ****************************************************************************
*                              EEPROM LOG
* ****************************************************************************
*/

static void eeprom_write_pulse_counters(void)
{
    eeprom_pulse_counters_0x03_t tmp;
    tmp.counter[0] = get_current_pulse_counter(0);
    tmp.counter[1] = get_current_pulse_counter(1);
    tmp.midnight_counter[0] = get_midnight_pulse_counter(0);
    tmp.midnight_counter[1] = get_midnight_pulse_counter(1);
    tmp.crc = crc8_compute((void*) &tmp, sizeof tmp - 1);
    nvm_eeprom_erase_and_write_buffer(offsetof(struct _eeprom_0x03_t, pulse_counters), &tmp, sizeof(tmp));
    nvm_wait_until_ready();
    DEBUG_putcrlf(__FUNCTION__);
}

static void eeprom_0x02_read_pulse_counters(void)
{
    uint8_t i;
    eeprom_pulse_counters_0x02_t tmp;

    nvm_eeprom_read_buffer(offsetof(struct _eeprom_0x02_t, pulse_counters_0x02), &tmp, sizeof tmp);
    if (crc8_compute((void*) &tmp, sizeof tmp) == 0) {
        for (i = 0; i < N_PULSE_COUNTERS; i++) {
            if (tmp.counter[i] == 0xFFFFFFFF) {
                tmp.counter[i] = 0;
            }
            if (tmp.midnight_counter[i] == 0xFFFFFFFF) {
                tmp.midnight_counter[i] = 0;
            }
        }
    } else {
        memset(&tmp, 0, sizeof tmp);
    }
    set_current_pulse_counter(0, tmp.counter[0]);
    set_current_pulse_counter(1, tmp.counter[1]);
    set_midnight_pulse_counter(0, tmp.midnight_counter[0]);
    set_midnight_pulse_counter(1, tmp.midnight_counter[1]);
}

static void eeprom_read_pulse_counters(void)
{
    uint8_t i;
    eeprom_pulse_counters_0x03_t tmp;
    DEBUG_putcrlf(__FUNCTION__);
    nvm_eeprom_read_buffer(offsetof(struct _eeprom_0x03_t, pulse_counters), &tmp, sizeof tmp);
    if (crc8_compute((void*) &tmp, sizeof tmp) == 0) {
        for (i = 0; i < N_PULSE_COUNTERS; i++) {
            if (tmp.counter[i] == 0xFFFFFFFF) {
                tmp.counter[i] = 0;
            }
            if (tmp.midnight_counter[i] == 0xFFFFFFFF) {
                tmp.midnight_counter[i] = 0;
            }
        }
    } else {
        memset(&tmp, 0, sizeof tmp);
    }
    set_current_pulse_counter(0, tmp.counter[0]);
    set_current_pulse_counter(1, tmp.counter[1]);
    set_midnight_pulse_counter(0, tmp.midnight_counter[0]);
    set_midnight_pulse_counter(1, tmp.midnight_counter[1]);
}

static void eeprom_write_config(void)
{
    DEBUG_putcrlf(__FUNCTION__);
    config.eeprom_version = CONFIG_EEPROM_VERSION;
    config.crc8 = crc8_compute((void*) &config, sizeof config - 1);
    cpu_irq_disable();
    nvm_eeprom_erase_and_write_buffer(offsetof(struct _eeprom_0x03_t, config), &config,
                                      sizeof config);
    nvm_wait_until_ready();
    cpu_irq_enable();
}

static void eeprom_read_setup(void)
{
    uint8_t i;

    memset(&config, 0, sizeof config);
    nvm_eeprom_read_buffer(offsetof(struct _eeprom_0x03_t, config), &config, sizeof config);

    switch (config.eeprom_version) {
    case 0x02:
        if (crc8_compute((void*) &config, sizeof(config_0x02_t)) != 0) {
            goto create_defaults;
        }
        for (i = 0; i < 3; i++) {
            config.pTimerWeekEndControl[i] = config.pTimerWeekDayControl[i];
            config.pTimerWeekDayWaterRunning[i] = 0;
            config.pTimerWeekEndWaterRunning[i] = 0;
        }
        eeprom_0x02_read_pulse_counters();
        eeprom_write_config();
        eeprom_write_pulse_counters();
        break;
    case CONFIG_EEPROM_VERSION:
        if (crc8_compute((void*) &config, sizeof(config_0x03_t)) != 0) {
            goto create_defaults;
        }
        eeprom_read_pulse_counters();
        break;
    default:
create_defaults:
        memset(&config, 0xFF, sizeof config);
        config.eeprom_version = CONFIG_EEPROM_VERSION;
        config.report_home_period_secs = CONFIG_DEFAULT_REPORT_HOME_PERIOD;
        config.rom[0].valid = false;
        config.rom[1].valid = false;
        config.rom[2].valid = false;
        config.temp_delta_trigger_x16 = CONFIG_DEFAULT_TEMP_DELTA_TRIG_X16;
        config.flow_delta_trigger = CONFIG_DEFAULT_FLOW_DELTA_TRIGGER;
        config.elec_delta_trigger = CONFIG_DEFAULT_ELEC_DELTA_TRIGGER;
        config.pTimerWeekDayControl[0] = 0;
        config.pTimerWeekDayControl[1] = 0;
        config.pTimerWeekDayControl[2] = 0;
        config.pTimerWeekEndControl[0] = 0;
        config.pTimerWeekEndControl[1] = 0;
        config.pTimerWeekEndControl[2] = 0;
        config.pTimerWeekDayWaterRunning[0] = 0;
        config.pTimerWeekDayWaterRunning[1] = 0;
        config.pTimerWeekDayWaterRunning[2] = 0;
        config.pTimerWeekEndWaterRunning[0] = 0;
        config.pTimerWeekEndWaterRunning[1] = 0;
        config.pTimerWeekEndWaterRunning[2] = 0;
        config.temp_sp_x16 = CONFIG_DEFAULT_TEMP_SP_X16;
        config.ctrl_select = CONTROL_SELECT_BYPASS;
        config.enable_temp1_hi_event = false;
        config.enable_temp1_lo_event = false;
        config.temp1_hi_trigger_x16 = CONFIG_DEFAULT_TEMP1_HI_TRIGGER_X16;
        config.temp1_lo_trigger_x16 = CONFIG_DEFAULT_TEMP1_LO_TRIGGER_X16;
        eeprom_write_config();
        eeprom_read_pulse_counters();
    }
}


/*
* ****************************************************************************
*                              CALL BACK HANDLERS
* ****************************************************************************
*/

static void modem_up_handler(bool up)
{
    state.modem.up = up;
}

static void modem_reset_handler(awt_reset_state_t reset_state)
{
    switch (reset_state) {
    case AWT_RESET_BUSY:
        break;
    case AWT_RESET_DONE:
        state.modem.secs_network_inactive = SECS_NETWORK_INACTIVE_TRIGGER / 2;  // see loop in main()
        state.modem.secs_since_last_report_home = config.report_home_period_secs;
        state.modem.up = false;
        state.modem.busy = false;
        break;
    }
}

void reset_avr(void);  // DO NOT CHANGE see uarte0 and uartc1

void reset_avr(void)
{
    eeprom_write_pulse_counters();
    Disable_global_interrupt();
    reset_do_soft_reset();
    wdt_reset_mcu();
    wdt_set_timeout_period(WDT_TIMEOUT_PERIOD_125CLK);
    wdt_enable();
    while (1) {
        ;
    }
}


/*
* ****************************************************************************
*                               EVENT BUFFER HANDLERS
* ****************************************************************************
*/

static void buffer_event(uint8_t event, const int32_t data)
{
    uint8_t b = (state.event_buffer.back + 1) % EVENT_BUFFER_SIZE;
    if (b != state.event_buffer.front) {
        event_t* pevent = & (state.event_buffer.buffer[state.event_buffer.back]);
        pevent->timestamp = epoch_get();
        pevent->event = event;
        pevent->is_int = true;
        pevent->int_data = data;
        state.event_buffer.back = b;
    }
}

static void buffer_str_event(uint8_t event, const char* data)
{
    uint8_t b = (state.event_buffer.back + 1) % EVENT_BUFFER_SIZE;
    if (b != state.event_buffer.front) {
        event_t* pevent = & (state.event_buffer.buffer[state.event_buffer.back]);
        pevent->timestamp = epoch_get();
        pevent->event = event;
        pevent->is_int = false;
        pevent->str_data = data;
        state.event_buffer.back = b;
    }
}

static void event_message_rsp(bool res)
{
    state.modem.busy = false;
    if (res) {
        state.modem.secs_network_inactive = 0;
        state.event_buffer.front = (state.event_buffer.front + 1) % EVENT_BUFFER_SIZE;
    }
}

static void send_event_message(void)
{
    char s[20];
    awt_event_t event;
    event_t* pevent = & (state.event_buffer.buffer[state.event_buffer.front]);
    event.timestamp = pevent->timestamp;
    event.code = pevent->event;
    if (pevent->is_int) {
        event.text = ltoa(pevent->int_data, s, 10);
    } else {
        event.text = pevent->str_data;
    }
    state.modem.busy = true;
    awt_dispatch_event_req(event_message_rsp, ASSET_ID, &event);
}


/*
* ****************************************************************************
*                               STATUS BUFFER HANDLERS
* ****************************************************************************
*/

static void buffer_current_status(void)
{
    uint8_t b = (state.status_buffer.back + 1) % STATUS_BUFFER_SIZE;
    if (b == state.status_buffer.front) {
        state.status_buffer.front = (b + 1) % STATUS_BUFFER_SIZE;
    }
    state.modem.secs_since_last_report_home = 0;
    state.status_buffer.buffer[state.status_buffer.back] = state.current;
    state.status_buffer.buffer[state.status_buffer.back].epoch = epoch_get();
    state.dispatched = state.status_buffer.buffer[state.status_buffer.back];
    state.status_buffer.back = b;
}

static void status_message_rsp(bool res)
{
    state.modem.busy = false;
    if (res) {
        state.modem.secs_network_inactive = 0;
        state.modem.secs_since_last_report_home %= config.report_home_period_secs;
        state.status_buffer.front = (state.status_buffer.front + 1) %
                                    STATUS_BUFFER_SIZE;
    }
}

static void send_status_message(void)
{
    awt_var_t var[13]; // +3];
    uint8_t count = 0;
    status_t* pbuf = &state.status_buffer.buffer[state.status_buffer.front];

    // 1
    var[count].name = "timestamp";
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->epoch;

    // 2
    var[count].name = STATUS_STATE_TAG;
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->state;

    // 3
    var[count].name = STATUS_POWER_ON_TAG;
    var[count].type = AWT_BOOL;
    var[count++].val.b = pbuf->power_on;

    // 4
    var[count].name = STATUS_ELEMENT_ON_TAG;
    var[count].type = AWT_BOOL;
    var[count++].val.b = pbuf->element_on;

    // 5
    var[count].name = STATUS_WATER_RUNNING_TAG;
    var[count].type = AWT_BOOL;
    var[count++].val.b = pbuf->water_event.running;

    // 6
    var[count].name = STATUS_PULSE1_TOTAL_TAG;
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->pulse_counter[0];

    // 7
    var[count].name = STATUS_PULSE2_TOTAL_TAG;
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->pulse_counter[1];

    // 8
    var[count].name = STATUS_PULSE1_RESET_TAG;
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->pulse_counter[0] - get_midnight_pulse_counter(0);

    // 9
    var[count].name = STATUS_PULSE2_RESET_TAG;
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->pulse_counter[1] - get_midnight_pulse_counter(1);


    // 10
    if (pbuf->temp_x16[0] != INT16_MIN) {
        var[count].name = STATUS_TEMP1_TAG;
        var[count].type = AWT_INT32;
        var[count++].val.i = pbuf->temp_x16[0];
    }

    // 11
    if (pbuf->temp_x16[1] != INT16_MIN) {
        var[count].name = STATUS_TEMP2_TAG;
        var[count].type = AWT_INT32;
        var[count++].val.i = pbuf->temp_x16[1];
    }

    // 12
    if (pbuf->temp_x16[2] != INT16_MIN) {
        var[count].name = STATUS_TEMP3_TAG;
        var[count].type = AWT_INT32;
        var[count++].val.i = pbuf->temp_x16[2];
    }

    // 13
    var[count].name = STATUS_EPOCH_TAG;
    var[count].type = AWT_INT32;
    var[count++].val.i = pbuf->epoch;


    state.modem.busy = true;
    awt_dispatch_data_req(status_message_rsp, STATUS_PATH, count, var);
}


/*
* ****************************************************************************
*                               CONFIG HANDLERS
* ****************************************************************************
*/


static void config_add_rom_code(uint8_t* rom_code, const char* rom_code_str)
{
    for (uint8_t k = 0; k < 3; k++) {
        if (!config.rom[k].valid) {
            memcpy(config.rom[k].code, rom_code, 8);
            config.rom[k].valid = true;
            eeprom_write_config();
            switch (k) {
            case 0:
                buffer_str_event(EVENT_NEW_TEMP1_SENSOR, rom_code_str);
                state.modem.send_rom_config = true;
                break;
            case 1:
                buffer_str_event(EVENT_NEW_TEMP2_SENSOR, rom_code_str);
                state.modem.send_rom_config = true;
                break;
            case 2:
                buffer_str_event(EVENT_NEW_TEMP3_SENSOR, rom_code_str);
                state.modem.send_rom_config = true;
                break;
            }
            return;
        }
    }
    DEBUG_putcrlf("ERROR ROM");
}

static void config_update_detected_sensors(void)
{
    for (uint8_t i = 0; i < ow_count; i++) {
        uint8_t k;
        for (k = 0; k < 3; k++) {
            if (config.rom[k].valid
                    && memcmp(ow_rom_codes[i], config.rom[k].code, 8) == 0) {
                break;
            }
        }
        if (k == 3) {
            config_add_rom_code(ow_rom_codes[i], ow_rom_codes_str[i]);
        }
    }
}

static int8_t config_find_ow_rom_code(uint8_t cfg_idx)
{
    uint8_t i;
    if (config.rom[cfg_idx].valid) {
        for (i = 0; i < ow_count; i++) {
            if (memcmp(config.rom[cfg_idx].code, ow_rom_codes[i], 8) == 0) {
                return i;
            }
        }
    }
    return -1;
}

static void config1_message_rsp(bool res)
{
    state.modem.busy = false;
    if (res) {
        state.modem.secs_network_inactive = 0;
        state.modem.send_config1 = false;
    } else {
        state.modem.send_config1 = true;
    }
}

static void config2_message_rsp(bool res)
{
    state.modem.busy = false;
    if (res) {
        state.modem.secs_network_inactive = 0;
        state.modem.send_config2 = false;
    } else {
        state.modem.send_config2 = true;
    }
}

static void rom_config_message_rsp(bool res)
{
    state.modem.busy = false;
    if (res) {
        state.modem.secs_network_inactive = 0;
        state.modem.send_rom_config = false;
    } else {
        state.modem.send_rom_config = true;
    }
}

static void send_config1_message(void)
{
    awt_var_t var[] = {
        {.name = CONFIG_REPORT_HOME_PERIOD_TAG, .type = AWT_INT32, .val.i = config.report_home_period_secs},
        {.name = CONFIG_TEMP_DELTA_TRIGGER_TAG, .type = AWT_INT32, .val.i = (config.temp_delta_trigger_x16 / MODEL_TEMP_SCALE_FACTOR) },
        {.name = CONFIG_ELEC_COUNTER_DELTA_TRIGGER_TAG, .type = AWT_INT32, .val.i = config.elec_delta_trigger},
        {.name = CONFIG_FLOW_COUNTER_DELTA_TRIGGER_TAG, .type = AWT_INT32, .val.i = config.flow_delta_trigger},
        {.name = CONFIG_CONTROL_TEMP_SP_TAG, .type = AWT_INT32, .val.i = (config.temp_sp_x16 / MODEL_TEMP_SCALE_FACTOR) },
        {.name = CONFIG_CONTROL_SELECT_TAG, .type = AWT_INT32, .val.i = config.ctrl_select},
        {.name = CONFIG_ENABLE_EVENT_HI_TAG, .type = AWT_BOOL, .val.b = config.enable_temp1_hi_event},
        {.name = CONFIG_ENABLE_EVENT_LO_TAG, .type = AWT_BOOL, .val.b = config.enable_temp1_lo_event},
        {.name = CONFIG_TEMP1_HI_TRIGGER_TAG, .type = AWT_INT32, .val.i = config.temp1_hi_trigger_x16 / MODEL_TEMP_SCALE_FACTOR},
        {.name = CONFIG_TEMP1_LO_TRIGGER_TAG, .type = AWT_INT32, .val.i = config.temp1_lo_trigger_x16 / MODEL_TEMP_SCALE_FACTOR}
    };
    state.modem.busy = true;
    awt_dispatch_data_req(config1_message_rsp, CONFIG_PATH,
                          sizeof var / sizeof var[0], var);
}

static void send_config2_message(void)
{
    awt_var_t var[] = {
        {.name = CONFIG_CONTROL_POOL_TIMER_1_WEEKDAY_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekDayControl[0]},
        {.name = CONFIG_CONTROL_POOL_TIMER_2_WEEKDAY_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekDayControl[1]},
        {.name = CONFIG_CONTROL_POOL_TIMER_3_WEEKDAY_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekDayControl[2]},
        {.name = CONFIG_CONTROL_POOL_TIMER_1_WEEKEND_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekEndControl[0]},
        {.name = CONFIG_CONTROL_POOL_TIMER_2_WEEKEND_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekEndControl[1]},
        {.name = CONFIG_CONTROL_POOL_TIMER_3_WEEKEND_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekEndControl[2]},
        {.name = CONFIG_WATER_RUNNING_POOL_TIMER_1_WEEKDAY_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekDayWaterRunning[0]},
        {.name = CONFIG_WATER_RUNNING_POOL_TIMER_2_WEEKDAY_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekDayWaterRunning[1]},
        {.name = CONFIG_WATER_RUNNING_POOL_TIMER_3_WEEKDAY_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekDayWaterRunning[2]},
        {.name = CONFIG_WATER_RUNNING_POOL_TIMER_1_WEEKEND_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekEndWaterRunning[0]},
        {.name = CONFIG_WATER_RUNNING_POOL_TIMER_2_WEEKEND_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekEndWaterRunning[1]},
        {.name = CONFIG_WATER_RUNNING_POOL_TIMER_3_WEEKEND_TAG, .type = AWT_INT32, .val.i = config.pTimerWeekEndWaterRunning[2]},
    };
    state.modem.busy = true;
    awt_dispatch_data_req(config2_message_rsp, CONFIG_PATH,
                          sizeof var / sizeof var[0], var);
}

static void send_rom_config_message(void)
{
    char code[3][17];
    awt_var_t var[] = {
        {.name = CONFIG_TEMP1_CODE_TAG, .type = AWT_STR, .val.s = code[0]},
        {.name = CONFIG_TEMP2_CODE_TAG, .type = AWT_STR, .val.s = code[1]},
        {.name = CONFIG_TEMP3_CODE_TAG, .type = AWT_STR, .val.s = code[2]},
    };
    for (uint8_t i = 0; i < 3; i++) {
        char* pchar = code[i];
        for (uint8_t j = 0; j < 8; j++) {
            *pchar++ = ram_hex[config.rom[i].code[j] / 16];
            *pchar++ = ram_hex[config.rom[i].code[j] % 16];
        }
        *pchar = '\x00';
    }
    state.modem.busy = true;
    awt_dispatch_data_req(rom_config_message_rsp, CONFIG_PATH,
                          sizeof var / sizeof var[0], var);
}

static void config_data_handler(uint32_t ticketid, const char* const path, uint8_t vars, const awt_var_t* var)
{
    bool update_eeprom = false;
    bool ack = true;
    state.modem.send_config1 = true;
    state.modem.send_config2 = true;
    for (uint8_t i = 0; ack && i < vars; i++, var++) {
        if (strcmp(var->name, CONFIG_REPORT_HOME_PERIOD_TAG) == 0 && var->type == AWT_INT32) {
            if (config.report_home_period_secs != var->val.i) {
                config.report_home_period_secs = var->val.i;
                if (config.report_home_period_secs < 60) {
                    config.report_home_period_secs = 60;
                } else if (config.report_home_period_secs >= 60 * 30) {
                    config.report_home_period_secs = 60 * 30;
                }
                update_eeprom = true;
            }
            //} else if (strcmp(var->name, CONFIG_TEMP1_CODE_TAG) == 0 && var->type == AWT_STR) {
            //} else if (strcmp(var->name, CONFIG_TEMP2_CODE_TAG) == 0 && var->type == AWT_STR) {    << READ ONLY
            //} else if (strcmp(var->name, CONFIG_TEMP3_CODE_TAG) == 0 && var->type == AWT_STR) {
        } else if (strcmp(var->name, CONFIG_TEMP_DELTA_TRIGGER_TAG) == 0 && var->type == AWT_INT32) {
            if (config.temp_delta_trigger_x16 != var->val.i * MODEL_TEMP_SCALE_FACTOR) {
                config.temp_delta_trigger_x16 = var->val.i * MODEL_TEMP_SCALE_FACTOR;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_ELEC_COUNTER_DELTA_TRIGGER_TAG) == 0 && var->type == AWT_INT32) {
            if (config.elec_delta_trigger != var->val.i) {
                config.elec_delta_trigger = var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_FLOW_COUNTER_DELTA_TRIGGER_TAG) == 0 && var->type == AWT_INT32) {
            if (config.flow_delta_trigger != var->val.i) {
                config.flow_delta_trigger = var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_POOL_TIMER_1_WEEKDAY_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekDayControl[0] != (uint32_t)(var->val.i)) {
                config.pTimerWeekDayControl[0] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_POOL_TIMER_2_WEEKDAY_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekDayControl[1] != (uint32_t)(var->val.i)) {
                config.pTimerWeekDayControl[1] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_POOL_TIMER_3_WEEKDAY_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekDayControl[2] != (uint32_t)(var->val.i)) {
                config.pTimerWeekDayControl[2] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_POOL_TIMER_1_WEEKEND_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekEndControl[0] != (uint32_t)(var->val.i)) {
                config.pTimerWeekEndControl[0] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_POOL_TIMER_2_WEEKEND_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekEndControl[1] != (uint32_t)(var->val.i)) {
                config.pTimerWeekEndControl[1] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_POOL_TIMER_3_WEEKEND_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekEndControl[2] != (uint32_t)(var->val.i)) {
                config.pTimerWeekEndControl[2] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_WATER_RUNNING_POOL_TIMER_1_WEEKDAY_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekDayWaterRunning[0] != (uint32_t)(var->val.i)) {
                config.pTimerWeekDayWaterRunning[0] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_WATER_RUNNING_POOL_TIMER_2_WEEKDAY_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekDayWaterRunning[1] != (uint32_t)(var->val.i)) {
                config.pTimerWeekDayWaterRunning[1] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_WATER_RUNNING_POOL_TIMER_3_WEEKDAY_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekDayWaterRunning[2] != (uint32_t)(var->val.i)) {
                config.pTimerWeekDayWaterRunning[2] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_WATER_RUNNING_POOL_TIMER_1_WEEKEND_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekEndWaterRunning[0] != (uint32_t)(var->val.i)) {
                config.pTimerWeekEndWaterRunning[0] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_WATER_RUNNING_POOL_TIMER_2_WEEKEND_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekEndWaterRunning[1] != (uint32_t)(var->val.i)) {
                config.pTimerWeekEndWaterRunning[1] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_WATER_RUNNING_POOL_TIMER_3_WEEKEND_TAG) == 0 && var->type == AWT_INT32) {
            if (config.pTimerWeekEndWaterRunning[2] != (uint32_t)(var->val.i)) {
                config.pTimerWeekEndWaterRunning[2] = (uint32_t) var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_TEMP_SP_TAG) == 0 && var->type == AWT_INT32) {
            if (config.temp_sp_x16 != (var->val.i * MODEL_TEMP_SCALE_FACTOR)) {
                config.temp_sp_x16 = var->val.i * MODEL_TEMP_SCALE_FACTOR;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_CONTROL_SELECT_TAG) == 0 && var->type == AWT_INT32) {
            if (config.ctrl_select != var->val.i) {
                config.ctrl_select = var->val.i;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_ENABLE_EVENT_HI_TAG) == 0 && var->type == AWT_BOOL) {
            if (config.enable_temp1_hi_event != var->val.b) {
                config.enable_temp1_hi_event = var->val.b;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_ENABLE_EVENT_LO_TAG) == 0 && var->type == AWT_BOOL) {
            if (config.enable_temp1_lo_event != var->val.b) {
                config.enable_temp1_lo_event = var->val.b;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_TEMP1_HI_TRIGGER_TAG) == 0 && var->type == AWT_INT32) {
            if (config.temp1_hi_trigger_x16 != (var->val.i * MODEL_TEMP_SCALE_FACTOR)) {
                config.temp1_hi_trigger_x16 = var->val.i * MODEL_TEMP_SCALE_FACTOR;
                update_eeprom = true;
            }
        } else if (strcmp(var->name, CONFIG_TEMP1_LO_TRIGGER_TAG) == 0 && var->type == AWT_INT32) {
            if (config.temp1_lo_trigger_x16 != (var->val.i * MODEL_TEMP_SCALE_FACTOR)) {
                config.temp1_lo_trigger_x16 = var->val.i * MODEL_TEMP_SCALE_FACTOR;
                update_eeprom = true;
            }
        } else {
            ack = false;
        }
    }
    if (ack) {
        if (update_eeprom) {
            eeprom_write_config();
            state.modem.secs_since_last_report_home = 0;
        }
        if (awt_buffer_ack_req(true, path, ticketid)) {
            state.modem.secs_network_inactive = 0;
        }
        state.modem.send_config1 = true;
        state.modem.send_config2 = true;
    }
    state.modem.send_config1 = true;
    state.modem.send_config2 = true;
}


/*
* ****************************************************************************
*
* ****************************************************************************
*/

static void id_message_rsp(bool res)
{
    state.modem.busy = false;
    if (res) {
        state.modem.secs_network_inactive = 0;
        state.modem.send_id = false;
    } else {
        state.modem.send_id = true;
    }
}

static void send_id_message(void)
{
    awt_var_t var[] = {
        {.name = STATUS_FIRMWARE_TAG, .type = AWT_STR, .val.s = STATUS_FIRMWARE_VERSION_STR},
        {.name = STATUS_RESET_REASON_TAG, .type = AWT_INT32, .val.i = state.reset_reason}
    };
    state.modem.busy = true;
    awt_dispatch_data_req(id_message_rsp, STATUS_PATH, sizeof var / sizeof var[0],
                          var);
}


/*
* ****************************************************************************
*
* ****************************************************************************
*/

static void data_handler(uint32_t ticketid, const char* const path,
                         uint8_t vars, const awt_var_t* var)
{
    if (strcmp(path, CONFIG_PATH) == 0) {
        config_data_handler(ticketid, path, vars, var);
    }
}

static uint8_t hex_nibble(int8_t nibble)
{
    nibble += 0x40;                             // add, al, 40H
    nibble += ((int16_t) nibble >> 8) & 0x09;   // cbw, and ah, 09H, add al, ah
    nibble &= 0x0f;                             // and al, 0FH
    return nibble;
}

static uint8_t hex_byte(const char* s)
{
    return (hex_nibble(*s) << 4) | hex_nibble(* (s + 1));
}

static bool valid_hex_char(uint8_t ch)
{
    return (ch >= '0' && ch <= '9') || (ch >= 'A' && ch <= 'F');
}

static void command_handler(uint32_t ticket_id, const char* const path,
                            const char* func_id, uint8_t n_params, const awt_param_t* param)
{
    if (strcmp(func_id, "reset") == 0) {
        awt_buffer_ack_req(true, path, ticket_id);
        timeout_stop(TIMEOUT_MAIN_EEPROM_SAVE_TIMER);
        timeout_start_singleshot(TIMEOUT_MAIN_EEPROM_SAVE_TIMER, TIMEOUT_TICK_HZ * 5);
        while (!timeout_test_and_clear_expired(TIMEOUT_MAIN_EEPROM_SAVE_TIMER)) {
            awt_send_any_enqueued_acks(update_asset_state);
        }
        if (param[0].val.b) {
            awt_reset_modem(NULL, __LINE__);
        }
        reset_avr();
    } else if (strcmp(func_id, "reset_pulse_counter") == 0) {
        if (n_params == 1 && param[0].type == AWT_INT32) {
            uint8_t mask = param[0].val.i;
            if (mask & 0x01) {
                set_current_pulse_counter(0, 0);
                set_midnight_pulse_counter(0, 0);
            }
            if (mask & 0x02) {
                set_current_pulse_counter(1, 0);
                set_midnight_pulse_counter(1, 0);
            }
            if (mask) {
                eeprom_write_pulse_counters();
                eeprom_read_pulse_counters();
            }
            awt_buffer_ack_req(true, path, ticket_id);
            return;
        }
    } else if (strcmp(func_id, "set_pulse_counter") == 0) {
        if (n_params == 2 && param[0].type == AWT_INT32 && param[1].type == AWT_INT32) {
            uint8_t counter = param[0].val.i;
            if (counter == 1 || counter == 2) {
                int32_t value = param[1].val.i;
                counter--;
                set_current_pulse_counter(counter, value);
                set_midnight_pulse_counter(counter, value);
                eeprom_write_pulse_counters();
                eeprom_read_pulse_counters();
                awt_buffer_ack_req(true, path, ticket_id);
                return;
            }
        }
    } else if (strcmp(func_id, "reset_defaults") == 0) {
        awt_buffer_ack_req(true, path, ticket_id);
        timeout_stop(TIMEOUT_MAIN_EEPROM_SAVE_TIMER);
        timeout_start_singleshot(TIMEOUT_MAIN_EEPROM_SAVE_TIMER, TIMEOUT_TICK_HZ * 5);
        while (!timeout_test_and_clear_expired(TIMEOUT_MAIN_EEPROM_SAVE_TIMER)) {
            awt_send_any_enqueued_acks(update_asset_state);
        }
        cpu_irq_disable();
        nvm_eeprom_erase_all();
        nvm_wait_until_ready();
        state.current.pulse_counter[0] = 0;
        state.current.pulse_counter[1] = 0;
        state.midnight_pulse_counter[0] = 0;
        state.midnight_pulse_counter[1] = 0;
        reset_avr();
    } else if (strcmp(func_id, "set_rom_id") == 0) {
        uint8_t i;
        if (n_params != 3) {
            goto send_nak;
        }
        if (param[0].type != AWT_STR || param[1].type != AWT_STR
                || param[2].type != AWT_STR) {
            goto send_nak;
        }
        if (strlen(param[0].val.s) != 16 || strlen(param[1].val.s) != 16
                || strlen(param[2].val.s) != 16) {
            goto send_nak;
        }
        for (i = 0; i < 16; i++) {
            if (!(valid_hex_char(param[0].val.s[i])  && valid_hex_char(param[1].val.s[i])
                    && valid_hex_char(param[2].val.s[i]))) {
                goto send_nak;
            }
        }
        config.rom[0].valid = false;
        config.rom[1].valid = false;
        config.rom[2].valid = false;
        for (i = 0; i < 8; i++) {
            config.rom[0].code[i] = hex_byte(& (param[0].val.s[i * 2]));
            config.rom[1].code[i] = hex_byte(& (param[1].val.s[i * 2]));
            config.rom[2].code[i] = hex_byte(& (param[2].val.s[i * 2]));
            config.rom[0].valid = config.rom[0].valid || config.rom[0].code[i] != 0xff;
            config.rom[1].valid = config.rom[1].valid || config.rom[1].code[i] != 0xff;
            config.rom[2].valid = config.rom[2].valid || config.rom[2].code[i] != 0xff;
        }
        eeprom_write_config();
        awt_buffer_ack_req(true, path, ticket_id);
        timeout_stop(TIMEOUT_MAIN_EEPROM_SAVE_TIMER);
        timeout_start_singleshot(TIMEOUT_MAIN_EEPROM_SAVE_TIMER, TIMEOUT_TICK_HZ * 10);
        while (!timeout_test_and_clear_expired(TIMEOUT_MAIN_EEPROM_SAVE_TIMER)) {
            awt_send_any_enqueued_acks(update_asset_state);
        }
        reset_avr();
    }
send_nak:
    awt_buffer_ack_req(false, path, ticket_id);
}


/*
* ****************************************************************************
*
* ****************************************************************************
*/


static void check_pulse_counters(void)
{
    state.count1.debounce = (state.count1.debounce << 1) | ioport_get_pin_level(GPIO_COUNT1);
    state.count1.debounce &= 0x07;
    if (state.count1.level == false)   {
        if (state.count1.debounce == 0x07) {
            state.count1.level = true;
            state.current.pulse_counter[ELEC_PULSE_COUNTER]++;
            state.geyser.elect.detected = true;
            state.geyser.elect.detected_at_epoch = epoch_get();
            ioport_toggle_pin(GPIO_LED1);
        }
    } else {
        if (state.count1.debounce == 0x00) {
            state.count1.level = false;
        }
    }

    state.count2.debounce = (state.count2.debounce << 1) | ioport_get_pin_level(GPIO_COUNT2);
    state.count2.debounce &= 0x07;
    if (state.count2.level == false)   {
        if (state.count2.debounce == 0x07) {
            state.count2.level = true;
            state.current.pulse_counter[FLOW_PULSE_COUNTER]++;
            state.geyser.water_running.detected = true;
            state.geyser.water_running.detected_at_epoch = epoch_get();
            ioport_toggle_pin(GPIO_LED2);
        }
    } else {
        if (state.count2.debounce == 0x00) {
            state.count2.level = false;
        }
    }
}

ISR(PORTD_INT0_vect)    // GPIO_COUNT2
{
    PORTD.INTFLAGS = 0x01;
}

ISR(PORTC_INT0_vect)    // GPIO_COUNT1
{
    PORTC.INTFLAGS = 0x01;
}

ISR(PORTB_INT0_vect)    // GPIO_COUNT3
{
    PORTB.INTFLAGS = 0x01;
}

ISR(PORTA_INT0_vect)    // GPIO_COUNT4
{
    PORTA.INTFLAGS = 0x01;
}

/*
* ****************************************************************************
* Rikus irrigation controller methods and decelerations
* ****************************************************************************
*/

void wipe_ica1_eeprom(int8_t timeslot)
{
    switch (timeslot) {
    case 0 : {
        for (int counter = 0; counter <= 0xF; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }
    case 1: {
        for (int counter = 10 ; counter <= 0x1F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }

    case 2: {
        for (int counter = 20 ; counter <= 0x2F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }
    }//switch
    return 0;
}//wipe_ica1_eeprom

void wipe_ica2_eeprom(int8_t timeslot)
{
    switch (timeslot) {
    case 0 : {
        for (int counter = 30; counter <= 0x3F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }
    case 1: {
        for (int counter = 40 ; counter <= 0x4F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }

    case 2: {
        for (int counter = 50 ; counter <= 0x5F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }
    }//switch
    return 0;
}//wipe_ica2_eeprom

void wipe_ica3_eeprom(int8_t timeslot)
{
    switch (timeslot) {
    case 0 : {
        for (int counter = 60; counter <= 0x6F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }
    case 1: {
        for (int counter = 70 ; counter <= 0x7F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }

    case 2: {
        for (int counter = 80 ; counter <= 0x8F; counter ++) {
            nvm_eeprom_write_byte(counter, 0xFF);
        }
        break;
    }
    }//switch
    return 0;
}//wipe_ica3_eeprom


void get_unsolicited_commands(int buf_size, char* buffer)
{
    char character;
    int count = 0;
    while (character != '\r' && character != '\n') {
        character = MODEM_get_byte();
        if (count < (buf_size - 1)) {
            buffer [count] = character;
        }
        count++;
    }//while
    buffer[buf_size - 1] = '\0';  //append null terminator
}

void clear_temp_string(int size, char* temp_buf)
{
    int temp_counter = 0;
    while (temp_counter < 60) {
        temp_buf[temp_counter] = NULL;
        temp_counter ++;
    }
}//clear_temp_string

void ack_job(int job_no)
{
    char ack [60];
    strcpy(ack, DUMMY);
    strcat(ack, job_no);
    MODEM_raw_puts(ack);
    MODEM_raw_putb('\n');
    MODEM_raw_putb('\r');
    delay_s(1.5);

    return 0;
}

void pwm_toggle(int channel, bool on, int duty, int frequency)
{
    struct pwm_config pwm_cfg_A;
    struct pwm_config pwm_cfg_B;
    struct pwm_config pwm_cfg_C;
    struct pwm_config pwm_cfg_D;
    switch (channel) {
    case 0: {
        pwm_init(&pwm_cfg_A, PWM_TCD0, PWM_CH_A, frequency); // PWM output channel 0
        if (on == true) {
            pwm_start(&pwm_cfg_A, duty); //duty
        } else {
            pwm_stop(&pwm_cfg_A);
        }
        break;
    }
    case 1: {
        pwm_init(&pwm_cfg_B, PWM_TCD0, PWM_CH_B, frequency); // PWM output channel 1
        if (on == true) {
            pwm_start(&pwm_cfg_B, duty); //duty
        } else {
            pwm_stop(&pwm_cfg_B);
        }
        break;
    }
    case 2: {
        pwm_init(&pwm_cfg_C, PWM_TCD0, PWM_CH_C, frequency); // PWM output channel 2
        if (on == true) {
            pwm_start(&pwm_cfg_C, duty); //duty
        } else {
            pwm_stop(&pwm_cfg_C);
        }
        break;
    }

    case 3: {
        pwm_init(&pwm_cfg_D, PWM_TCD0, PWM_CH_D, frequency); // PWM output channel 3
        if (on == true) {
            pwm_start(&pwm_cfg_D, duty); //duty
        } else {
            pwm_stop(&pwm_cfg_D);
        }
        break;
    }

    }//switch

    return 0;
}

asset_model* decode(char* serial_in)
{
    int delimiters_array [] = {0, 0, 0, 0, 0, 0, 0};
    int pos = 0;
    char* token = strtok(serial_in, ": ,");
    static asset_model model;

    if (strstr(serial_in, "+RTC") != NULL) { //Get current real time and set RTC
        char time_token[8];
        uint32_t hours;
        uint32_t minutes;
        uint32_t seconds;
        for (int x = 16; x <= 23; x++) { //get the time from at-command
            time_token[x - 16] = (char) serial_in[x];
        }
        hours = ((uint32_t)(time_token[0] - '0')) * 10 + (uint32_t)(time_token[1] - '0');
        minutes = ((uint32_t)(time_token[3] - '0')) * 10 + (uint32_t)(time_token[4] - '0');
        seconds = ((uint32_t)(time_token[6] - '0')) * 10 + (uint32_t)(time_token[7] - '0');
        uint32_t rtc_seconds = (hours * 3600) + (minutes * 60) + seconds;
        rtc_set_time(rtc_seconds);
    }

    while (token != NULL) {
        token = strtok(NULL, ",");
        switch (pos) {
        case 0:
            model.job_type = token;
            break;
        case 1:
            ack_job(token);
            model.job_id = (int32_t) token;
            break;
        case 2:
            model.asset_id = token;
            break;
        case 3:
            model.job_desc = token;
            break;
        case 4:
            model.data_type = token;
            break;
        case 5:
            model.job_value = (int32_t) token;
            break;
        case 6:
            model.crc = token;
            break;
        }
        pos++;
    }
    return &model; //return address of the model struct
}

void clear_eeprom(int addr_start, int addr_end)
{
    for (int start = addr_start; start <= addr_end; start ++) {
        nvm_eeprom_write_byte(start, 0xFF);
    }
    return 0;
}

void schedule_job(asset_model* model)
{
    int count = 0;
    int address_count = 0x0000;
    char job_value_buffer[15];

    if (strcmp(model->asset_id, "ICA1") == 0) {
        if (strcmp(model->job_desc, "TV_ts0") == 0) {
            clear_eeprom(0x0000, 0x000F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "TV_ts1") == 0) {
            address_count = 0x0010;
            clear_eeprom(address_count, 0x001F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "TV_ts2") == 0) {
            address_count = 0x0020;
            clear_eeprom(address_count, 0x002F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "CM_ts0") == 0) {
            //check int value for confirmation that reset eeprom has been set
            wipe_ica1_eeprom(0);
        }
        if (strcmp(model->job_desc, "CM_ts1") == 0) {
            wipe_ica1_eeprom(1);
        }
        if (strcmp(model->job_desc, "CM_ts2") == 0) {
            wipe_ica1_eeprom(2);
        }
    }//ICA1
    if (strcmp(model->asset_id, "ICA2") == 0) {
        if (strcmp(model->job_desc, "TV_ts0") == 0) {
            address_count = 0x0030;
            clear_eeprom(address_count, 0x003F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "TV_ts1") == 0) {
            address_count = 0x0040;
            clear_eeprom(address_count, 0x004F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "TV_ts2") == 0) {
            address_count = 0x0050;
            clear_eeprom(address_count, 0x005F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "CM_ts0") == 0) {
            wipe_ica2_eeprom(0);
        }
        if (strcmp(model->job_desc, "CM_ts1") == 0) {
            wipe_ica2_eeprom(1);
        }
        if (strcmp(model->job_desc, "CM_ts2") == 0) {
            wipe_ica2_eeprom(2);
        }
    }//ICA2
    if (strcmp(model->asset_id, "ICA3") == 0) {
        if (strcmp(model->job_desc, "TV_ts0") == 0) {
            address_count = 0x0060;
            clear_eeprom(address_count, 0x006F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "TV_ts1") == 0) {
            address_count = 0x0070;
            clear_eeprom(address_count, 0x007F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "TV_ts2") == 0) {
            address_count = 0x0080;
            clear_eeprom(address_count, 0x008F);
            strcpy(job_value_buffer, model->job_value);
            while (job_value_buffer[count] != NULL) {
                nvm_eeprom_write_byte(address_count, job_value_buffer[count]);
                count++;
                address_count++;
            }
            count = 0;
        }
        if (strcmp(model->job_desc, "CM_ts0") == 0) {
            wipe_ica3_eeprom(0);
        }
        if (strcmp(model->job_desc, "CM_ts1") == 0) {
            wipe_ica3_eeprom(1);
        }
        if (strcmp(model->job_desc, "CM_ts2") == 0) {
            wipe_ica3_eeprom(2);
        }
    }//ICA3

    return 0;
}

void print_eeprom(void)
{
    int addr = 0x0000;
    while (addr < 0x0010) {
        DEBUG_puts("eeprom : ");
        _DEBUG_putc((char)nvm_eeprom_read_byte(addr));
        DEBUG_puts("\n \r");
        addr++;
    }
    return 0;
}

void int_to_binary(int32_t k, int8_t bin_array[32])
{
    int8_t count = 0;
    int32_t value = k;
    DEBUG_puts("k : ");
    DEBUG_putu(k);
    DEBUG_puts("\n \r");
    while (value != 0 && value != 1) {
        bin_array[count] = k % 2;
        count++;
        value = k / 2 ;
        k = value;
        if (value == 1) {
            bin_array[count] = 1;
            count++;
        }
    }
    return 0;
}//int_to_binary



void execute_jobs(char* scheduled_char_time_ptr, int8_t size, char* asset)
{
    int32_t job_value = 0;
    int8_t array_pos = 0;
    int8_t bin_array_pass [32] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t current_time = rtc_get_time(); //seconds
    uint32_t start_time = 0; //seconds
    uint32_t end_time = 0; //seconds
    uint32_t timeslot_offset = 0;
    int8_t next_bin_value = 0; // check the next value of the binary array to check if next job has been set to not stop the current job.
    bool next_job = false;

    for (int x = (size - 1); x >= 0; x--) {
        if ((Byte)(scheduled_char_time_ptr) != 0xFF) {
            int32_t char_byte = (int32_t)(scheduled_char_time_ptr[x] - '0');
            switch (array_pos) {
            case 0 : {
                job_value = job_value + char_byte;
                break;
            }
            case 1: {
                job_value = job_value + 10 * char_byte;
                break;
            }
            case 2: {
                job_value = job_value + 100 * char_byte;
                break;
            }
            case 3: {
                job_value = job_value + 1000 * char_byte;
                break;
            }
            case 4: {
                job_value = job_value + 10000 * char_byte;
                break;
            }
            case 5: {
                job_value = job_value + 100000 * char_byte;
                break;
            }
            case 6: {
                job_value = job_value + 1000000 * char_byte;
                break;
            }
            case 7: {
                job_value = job_value + 10000000 * char_byte;
                break;
            }
            case 8: {
                job_value = job_value + 100000000 * char_byte;
                break;
            }
            case 9: {
                job_value = job_value + 1000000000 * char_byte;
                break;
            }
            case 10: {
                job_value = job_value + 10000000000 * char_byte;
                break;
            }
            }//switch
            array_pos++;
        }//if
    }//for

    if (job_value < 0) { //if signed int32 value has overrun, then it is modified to represent the correct binary value
        uint32_t temp_value = -1 * job_value + 2147483648;
        job_value = temp_value;
    }

    int_to_binary(job_value, bin_array_pass);
    current_time = rtc_get_time();

    if (asset == "ICA1_ts0") {
        timeslot_offset = 0;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA1_ts0!!! \n \r");
                    pwm_toggle(0, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA1_ts0!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(0, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA1_ts1") {
        timeslot_offset = 8 * 60 * 60L;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA1_ts1!!! \n \r");
                    pwm_toggle(0, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA1_ts1!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(0, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA1_ts2") {
        timeslot_offset = 16 * 60 * 60L; //change to L for all the rest multiplications so that the sum works correctly
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA1_ts2!!! \n \r");
                    pwm_toggle(0, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA1_ts2!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(0, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA2_ts0") {
        timeslot_offset = 0;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA2_ts0!!! \n \r");
                    pwm_toggle(1, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA2_ts0!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(1, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA2_ts1") {
        timeslot_offset = 8 * 60 * 60L;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA2_ts1!!! \n \r");
                    pwm_toggle(1, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA2_ts1!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(1, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA2_ts2") {
        timeslot_offset = 16 * 60 * 60L;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA2_ts2!!! \n \r");
                    pwm_toggle(1, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA2_ts2!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(1, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA3_ts0") {
        timeslot_offset = 0;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA3_ts0!!! \n \r");
                    pwm_toggle(3, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA3_ts0!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(3, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA3_ts1") {
        timeslot_offset = 8 * 60 * 60L;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA3_ts1!!! \n \r");
                    pwm_toggle(3, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA3_ts1!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(3, false, 0, 0);
                    }
                }

            }
        }
    }
    if (asset == "ICA3_ts2") {
        timeslot_offset = 16 * 60 * 60L;
        for (int arr_index = 0; arr_index < 32; arr_index ++) {
            if (bin_array_pass[arr_index] == 1) {
                start_time = arr_index * (15 * 60) + timeslot_offset;
                end_time = start_time + (15 * 60);
                if ((arr_index + 1) <= 31) {
                    next_bin_value = bin_array_pass[arr_index + 1];
                    if (next_bin_value == 1) {
                        next_job = true;
                    } else {
                        next_job = false;
                    }
                }
                if (current_time  >= start_time && current_time <= (start_time + 60)) {
                    //start pwm method
                    DEBUG_puts("!!!start pwm method-ICA3_ts2!!! \n \r");
                    pwm_toggle(3, true, 10, 10);
                }
                if (current_time >= end_time && current_time <= (end_time + 60)) {
                    if (next_job == false) {
                        DEBUG_puts("!!!end pwm method-ICA3_ts2!!! \n \r"); //////////complete for all ifs
                        pwm_toggle(3, false, 0, 0);
                    }
                }

            }
        }
    }
    for (int x = 0; x < 32 ; x++) { //clear bin_array_pass
        bin_array_pass[x] = 0;
    }

}//execute_jobs
void look_for_jobs(void)
{
    //check eeprom for scheduled jobs and execute them on the minute
    uint32_t current_time = rtc_get_time();
    int8_t count = 0;
    static char scheduled_char_time[16];
    char eeprom_char_byte;
    for (int address = 0x0000; address <= 0x008F; address++) { //loop through entire eeprom address range
        if (address <= 0x000F) { //ICA1_ts0
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x000F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA1_ts0");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA1_ts0
        if (address >= 0x0010 && address <= 0x001F) { //ICA1_ts1
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x001F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA1_ts1");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA1_ts1
        if (address >= 0x0020 && address <= 0x002F) { //ICA1_ts2
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x002F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA1_ts2");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA1_ts2
        if (address >= 0x0030 && address <= 0x003F) { //ICA2_ts0
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x003F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA2_ts0");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA2_ts0
        if (address >= 0x0040 && address <= 0x004F) { //ICA2_ts1
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x004F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA2_ts1");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0x00FF);
                }
                count = 0;
            }
        }//ICA2_ts1
        if (address >= 0x0050 && address <= 0x005F) { //ICA2_ts2
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x005F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA2_ts2");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }
        if (address >= 0x0060 && address <= 0x006F) { //ICA3_ts0
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x006F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA3_ts0");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA3_ts0
        if (address >= 0x0070 && address <= 0x007F) { //ICA3_ts1
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x007F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA3_ts1");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA3_ts1
        if (address >= 0x0080 && address <= 0x008F) { //ICA3_ts2
            eeprom_char_byte = (char) nvm_eeprom_read_byte(address);
            if ((Byte) eeprom_char_byte != 0xFF) {
                scheduled_char_time[count] = eeprom_char_byte;
                count ++;
            }
            if (address == 0x008F) {
                char* scheduled_char_time_ptr = &scheduled_char_time;
                execute_jobs(scheduled_char_time_ptr, count, "ICA3_ts2");
                for (int x = 0; x <= 15; x++) { //clear array
                    scheduled_char_time[x] = (char)(0xFF);
                }
                count = 0;
            }
        }//ICA3_ts2
    }//loop through entire eeprom
    return 0;
}

int main(void)
{
    irq_initialize_vectors();
    pmic_init();
    sysclk_init();
    timeout_init();
    ioport_init();
    board_init();
    sleepmgr_init();
    rtc_init();
    cpu_irq_enable();
    delay_init(F_CPU);

    ioport_set_pin_low(GPIO_LED1);
    _DEBUG_init();
    _DEBUG_enable_interrupt(USART_INT_LVL_LO);
    _MODEM_init();
    _MODEM_enable_interrupt(USART_INT_LVL_LO);
    char unsolicited_command[60];
    char* unsolicited_command_ptr = &unsolicited_command;
    uint32_t dummy_timer;
    uint32_t rtc_timer;
    uint32_t look_for_jobs_timer;
    uint32_t system_time_timer; //

    MODEM_raw_puts("at+awtda=c*");
    MODEM_raw_putb('\r');
    MODEM_raw_putb('\n');
    delay_s(1.5);
    MODEM_raw_puts("at+awtda=d*");
    MODEM_raw_putb('\r');
    MODEM_raw_putb('\n');
    delay_s(1.5);
    MODEM_raw_puts(RTC_TIME);
    MODEM_raw_putb('\r');
    MODEM_raw_putb('\n');
    delay_s(1.5);

    //pwm_toggle(0, true, 10, 10);
    //delay_s(5);
    //pwm_toggle(0, false, 0, 0);

    dummy_timer = rtc_get_time();
    rtc_timer = rtc_get_time();
    while (1) {
        if ((rtc_get_time() - dummy_timer) >= 4) {
            MODEM_raw_puts(DUMMY_WRITE);
            MODEM_raw_putb('\n');
            MODEM_raw_putb('\r');
            delay_s(1.5);
            dummy_timer = rtc_get_time(); //update timer
        }

        if (_uarte0_byte_available() == true) { //modem uart gets unsolicted interrupt
            unsolicited_command_ptr = &unsolicited_command;
            get_unsolicited_commands(sizeof(unsolicited_command), unsolicited_command_ptr);
            if (unsolicited_command[0] != '\n' && unsolicited_command[0] != '\r' && unsolicited_command[0] == '+') {
                DEBUG_puts(unsolicited_command);
                _DEBUG_putc('\n');
                _DEBUG_putc('\r');
                asset_model* model_ptr = decode(unsolicited_command_ptr);
                schedule_job(model_ptr);
                //print_eeprom();
            }
            clear_temp_string(sizeof(unsolicited_command_ptr), unsolicited_command_ptr);
        }
        if ((rtc_get_time() - rtc_timer) >= 10) { //check currrent time
            DEBUG_puts("Time : ");
            uint32_t time = rtc_get_time();
            DEBUG_putu(time);
            DEBUG_puts("\n \r");
            rtc_timer = rtc_get_time(); //update timer
        }
        if (rtc_get_time() - look_for_jobs_timer >= 20) {
            look_for_jobs();
            look_for_jobs_timer = rtc_get_time(); //update timer
        }
        if (rtc_get_time() - system_time_timer >= 60) { //update system time every minute
            MODEM_raw_puts(RTC_TIME);
            MODEM_raw_putb('\r');
            MODEM_raw_putb('\n');
            delay_s(1.5);
            system_time_timer = rtc_get_time(); //update timer
        }
    }//while(1)
    return 0;
}//main
