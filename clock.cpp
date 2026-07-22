/**
 * @file clock.cpp
 * @author Benjamin Hartmann
 * @brief Main program for the Raspberry Pi Pico clock project.
 * @version 0.0.1
 * @date 2026-07-19
 *
 * @copyright Copyright (c) 2026 Benjamin Hartmann
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>

#include "hardware/clocks.h"
#include "hardware/i2c.h"
#include "hardware/pwm.h"
#include "hardware/sync.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "pico/util/datetime.h"

extern "C" {
#include <PicoTM1637.h>

#include "ds3231.h"
#include "ssd1306.h"
}

#define CLOCK_FIRMWARE_VERSION "0.0.1"

#define SEVEN_SEG_CLK_PIN 2
#define SEVEN_SEG_DIO_PIN 3
#define I2C_PORT i2c1
#define I2C_SDA 6
#define I2C_SCL 7
#define I2C_BAUDRATE (100 * 1000)
#define I2C_SCAN_TIMEOUT_US 1000
#define I2C_WIDTH 128
#define I2C_HEIGHT 64
#define I2C_OLED_ADDRESS 0x3C
#define I2C_DS3231_ADDRESS 0x68
#define BUZZER_PIN 4
#define ROTARY_CLK_PIN 10
#define ROTARY_DT_PIN 11
#define ROTARY_SW_PIN 12
#define ROTARY_COUNTS_PER_DETENT 4
#define ROTARY_BUTTON_DEBOUNCE_US 20000
#define SERIAL_LINE_LENGTH 32
#define BUZZER_PWM_DIVIDER 100.0f

static bool twentyfour_hour = true;
static uint8_t alarm_hour = 6;
static uint8_t alarm_minute = 30;
static bool alarm_enabled = true;
static uint8_t alarm_duration_seconds = 30;

ssd1306_t disp;
ds3231_t rtc;

static volatile int32_t rotary_pending_delta = 0;
static volatile bool rotary_release_pending = false;
static uint8_t rotary_previous_state = 0;
static int8_t rotary_transition_count = 0;
static uint32_t rotary_last_button_us = 0;

enum serial_ui_state_t {
    SERIAL_UI_IDLE,
    SERIAL_UI_MENU,
    SERIAL_UI_SET_YEAR,
    SERIAL_UI_SET_MONTH,
    SERIAL_UI_SET_DATE,
    SERIAL_UI_SET_HOUR,
    SERIAL_UI_SET_MINUTE,
    SERIAL_UI_SET_SECOND,
    SERIAL_UI_SET_FORMAT,
    SERIAL_UI_SET_ALARM,
    SERIAL_UI_SET_DURATION,
    SERIAL_UI_SET_ENABLED,
};

static serial_ui_state_t serial_ui_state = SERIAL_UI_IDLE;
static char serial_line[SERIAL_LINE_LENGTH];
static size_t serial_line_length = 0;
static ds3231_data_t pending_time = {};

static uint buzzer_slice;
static uint buzzer_channel;
static bool alarm_sounding = false;
static bool buzzer_tone_on = false;
static size_t melody_note = 0;
static absolute_time_t melody_note_off_time;
static absolute_time_t melody_next_note_time;

static const uint16_t alarm_melody[] = {330, 392, 330, 262, 294, 196, 294, 330};

static const int8_t rotary_transition_table[16] = {
    0, -1, 1, 0, 1, 0, 0, -1, -1, 0, 0, 1, 0, 1, -1, 0,
};

int pico_led_init();
void pico_set_led(bool);
int pico_tm1637_init();
int pico_i2c_init();
int pico_ssd1306_init();
int buzzer_init();
int rotary_encoder_init();
void handle_rotary_encoder();
void sleep_with_rotary_encoder(uint32_t duration_ms);
void bus_scan();
void display_time(const ds3231_data_t* now);
void print_time(const ds3231_data_t* now);
void serial_ui_poll();
void update_alarm(const ds3231_data_t* now);
bool set_rtc_to_build_time();

/**
 * @brief Main entry point for the program.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int main() {
    stdio_init_all();
    hard_assert(pico_led_init() == PICO_OK);
    printf("------- Starting clock firmware version: %s -------\n",
           CLOCK_FIRMWARE_VERSION);

    hard_assert(pico_tm1637_init() == PICO_OK);
    hard_assert(pico_i2c_init() == PICO_OK);
    hard_assert(pico_ssd1306_init() == PICO_OK);
    hard_assert(rotary_encoder_init() == PICO_OK);
    hard_assert(buzzer_init() == PICO_OK);

    ds3231_init(&rtc, I2C_PORT, I2C_DS3231_ADDRESS, 0);
    const int oscillator_status = ds3231_check_oscillator_stop_flag(&rtc);
    if (oscillator_status < 0) {
        printf("Could not find DS3231");
    } else if (oscillator_status > 0) {
        printf("DS3231 oscillator-stop flag is set; setting build time\n");
        if (!set_rtc_to_build_time()) {
            printf("Could not set DS3231 build time\n");
        }
    }

    for (int i = 0; i < 5; i++) {
        pico_set_led(true);
        sleep_ms(100);
        pico_set_led(false);
        sleep_ms(100);
    }

    char title[] = "CLK";
    TM1637_display_word(title, true);

    bus_scan();

    printf("Alarm Clock\n");
    printf("Commands: 't' status, 's' settings\n");

    ds3231_data_t now = {};
    uint8_t previous_second = UINT8_MAX;
    absolute_time_t next_rtc_read = get_absolute_time();

    while (true) {
        handle_rotary_encoder();
        serial_ui_poll();

        if (time_reached(next_rtc_read)) {
            next_rtc_read = make_timeout_time_ms(100);
            if (ds3231_read_current_time(&rtc, &now) == 0) {
                if (now.seconds != previous_second) {
                    display_time(&now);
                    previous_second = now.seconds;
                }
                update_alarm(&now);
            } else {
                update_alarm(nullptr);
            }
        } else if (alarm_sounding) {
            update_alarm(&now);
        }

        sleep_ms(1);
    }
}

/**
 * @brief Initialize the Pico LED.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int pico_led_init() { return cyw43_arch_init(); }

/**
 * @brief Initialize the TM1637 display.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int pico_tm1637_init() {
    TM1637_init(SEVEN_SEG_CLK_PIN, SEVEN_SEG_DIO_PIN);
    TM1637_clear();
    TM1637_set_brightness(7);  // max value, default is 0
    return PICO_OK;
}

/**
 * @brief Initialize and clear the SSD1306 OLED display.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int pico_ssd1306_init() {
    ssd1306_init(&disp, I2C_WIDTH, I2C_HEIGHT, I2C_OLED_ADDRESS, I2C_PORT);
    ssd1306_clear(&disp);
    return PICO_OK;
}

/**
 * @brief Read the current CLK and DT input state.
 *
 * @return The two-bit encoder state, with CLK as the most significant bit.
 */
static uint8_t rotary_encoder_state() {
    const uint32_t pins = gpio_get_all();
    return (uint8_t)((((pins >> ROTARY_CLK_PIN) & 1u) << 1) |
                     ((pins >> ROTARY_DT_PIN) & 1u));
}

/**
 * @brief Decode rotary transitions and record debounced switch releases.
 *
 * @param gpio GPIO pin that generated the interrupt.
 * @param events Bit mask of GPIO interrupt events.
 */
static void rotary_encoder_irq(uint gpio, uint32_t events) {
    if (gpio == ROTARY_CLK_PIN || gpio == ROTARY_DT_PIN) {
        const uint8_t new_state = rotary_encoder_state();
        const uint8_t transition =
            (uint8_t)((rotary_previous_state << 2) | new_state);

        rotary_transition_count += rotary_transition_table[transition];
        rotary_previous_state = new_state;

        if (rotary_transition_count >= ROTARY_COUNTS_PER_DETENT) {
            rotary_pending_delta++;
            rotary_transition_count = 0;
        } else if (rotary_transition_count <= -ROTARY_COUNTS_PER_DETENT) {
            rotary_pending_delta--;
            rotary_transition_count = 0;
        }
    }

    if (gpio == ROTARY_SW_PIN && (events & GPIO_IRQ_EDGE_RISE)) {
        const uint32_t now = time_us_32();
        if ((uint32_t)(now - rotary_last_button_us) >=
            ROTARY_BUTTON_DEBOUNCE_US) {
            rotary_release_pending = true;
            rotary_last_button_us = now;
        }
    }
}

/**
 * @brief Initialize the KY-040 rotary encoder and its push switch.
 *
 * CLK, DT, and SW are active-low inputs using the Pico's internal pull-ups.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int rotary_encoder_init() {
    const uint rotary_pins[] = {
        ROTARY_CLK_PIN,
        ROTARY_DT_PIN,
        ROTARY_SW_PIN,
    };

    for (uint pin : rotary_pins) {
        gpio_init(pin);
        gpio_set_dir(pin, GPIO_IN);
        gpio_pull_up(pin);
    }

    rotary_previous_state = rotary_encoder_state();

    const uint32_t quadrature_edges = GPIO_IRQ_EDGE_RISE | GPIO_IRQ_EDGE_FALL;
    gpio_set_irq_enabled_with_callback(ROTARY_CLK_PIN, quadrature_edges, true,
                                       rotary_encoder_irq);
    gpio_set_irq_enabled(ROTARY_DT_PIN, quadrature_edges, true);
    gpio_set_irq_enabled(ROTARY_SW_PIN, GPIO_IRQ_EDGE_RISE, true);

    bi_decl(bi_3pins_with_names(ROTARY_CLK_PIN, "Rotary encoder CLK",
                                ROTARY_DT_PIN, "Rotary encoder DT",
                                ROTARY_SW_PIN, "Rotary encoder switch"));
    return PICO_OK;
}

/**
 * @brief Report pending rotary encoder events outside interrupt context.
 */
void handle_rotary_encoder() {
    const uint32_t interrupt_state = save_and_disable_interrupts();
    const int32_t delta = rotary_pending_delta;
    const bool button_released = rotary_release_pending;
    rotary_pending_delta = 0;
    rotary_release_pending = false;
    restore_interrupts(interrupt_state);

    for (int32_t remaining = delta; remaining > 0; --remaining) {
        printf("Rotary encoder: +1\n");
    }

    for (int32_t remaining = delta; remaining < 0; ++remaining) {
        printf("Rotary encoder: -1\n");
    }

    if (button_released) {
        printf("Rotary encoder button released\n");
    }
}

/**
 * @brief Sleep while continuing to dispatch pending rotary encoder events.
 *
 * @param duration_ms Sleep duration in milliseconds.
 */
void sleep_with_rotary_encoder(uint32_t duration_ms) {
    const absolute_time_t end = make_timeout_time_ms(duration_ms);
    while (!time_reached(end)) {
        handle_rotary_encoder();
        sleep_ms(1);
    }
}

/**
 * @brief Initialize the I2C bus.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int pico_i2c_init() {
    i2c_init(I2C_PORT, I2C_BAUDRATE);
    gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
    gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
    gpio_pull_up(I2C_SCL);
    gpio_pull_up(I2C_SDA);
    // Make the I2C pins available to picotool
    bi_decl(bi_2pins_with_func(I2C_SDA, I2C_SCL, GPIO_FUNC_I2C));
    return PICO_OK;
}

/**
 * @brief Set the state of the Pico LED.
 *
 * @param led_on true to turn the LED on, false to turn it off
 */
void pico_set_led(bool led_on) {
    cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, led_on);
}

/**
 * @brief Determine whether an I2C address is reserved.
 *
 * Reserved addresses have the form 000 0xxx or 111 1xxx and are excluded from
 * the bus scan.
 *
 * @param addr The I2C address to check.
 * @return true if the address is reserved, false otherwise.
 */
bool is_reserved_addr(uint8_t addr) {
    return (addr & 0x78) == 0 || (addr & 0x78) == 0x78;
}

/**
 * @brief Sweep through all 7-bit I2C addresses, to see if any slaves are
 * present on the I2C bus. Print out a table.
 */
void bus_scan() {
    printf("\nI2C Bus Scan\n");
    printf("   0  1  2  3  4  5  6  7  8  9  A  B  C  D  E  F\n");

    for (int addr = 0; addr < (1 << 7); ++addr) {
        if (addr % 16 == 0) {
            printf("%02x ", addr);
        }

        // Perform a 1-byte dummy read from the probe address. If a slave
        // acknowledges this address, the function returns the number of bytes
        // transferred. If the address byte is ignored, the function returns
        // -1.

        // Skip over any reserved addresses.
        int ret;
        uint8_t rxdata;
        if (is_reserved_addr(addr))
            ret = PICO_ERROR_GENERIC;
        else {
            ret = i2c_read_timeout_us(I2C_PORT, addr, &rxdata, 1, false,
                                      I2C_SCAN_TIMEOUT_US);
        }

        printf(ret < 0 ? "." : "@");
        printf(addr % 16 == 15 ? "\n" : "  ");
    }
    printf("Done.\n\n");
}

/**
 * @brief Initialize the piezo buzzer PWM output.
 */
int buzzer_init() {
    gpio_set_function(BUZZER_PIN, GPIO_FUNC_PWM);
    buzzer_slice = pwm_gpio_to_slice_num(BUZZER_PIN);
    buzzer_channel = pwm_gpio_to_channel(BUZZER_PIN);
    pwm_set_clkdiv(buzzer_slice, BUZZER_PWM_DIVIDER);
    pwm_set_chan_level(buzzer_slice, buzzer_channel, 0);
    pwm_set_enabled(buzzer_slice, true);
    bi_decl(bi_1pin_with_name(BUZZER_PIN, "Piezo buzzer"));
    return PICO_OK;
}

static void buzzer_stop_tone() {
    pwm_set_chan_level(buzzer_slice, buzzer_channel, 0);
    buzzer_tone_on = false;
}

static void buzzer_start_tone(uint32_t frequency_hz) {
    const uint32_t pwm_clock_hz =
        (uint32_t)(clock_get_hz(clk_sys) / BUZZER_PWM_DIVIDER);
    const uint16_t wrap = (uint16_t)(pwm_clock_hz / frequency_hz - 1u);
    pwm_set_wrap(buzzer_slice, wrap);
    pwm_set_chan_level(buzzer_slice, buzzer_channel, (wrap + 1u) / 2u);
    pwm_set_counter(buzzer_slice, 0);
    buzzer_tone_on = true;
}

static void start_alarm_note() {
    buzzer_start_tone(alarm_melody[melody_note]);
    melody_note_off_time = make_timeout_time_ms(250);
    melody_next_note_time = make_timeout_time_ms(325);
}

static void stop_alarm() {
    buzzer_stop_tone();
    alarm_sounding = false;
    melody_note = 0;
}

/**
 * @brief Start, advance, or stop the non-blocking alarm melody.
 */
void update_alarm(const ds3231_data_t* now) {
    const bool in_alarm_window =
        now != nullptr && alarm_enabled && now->hours == alarm_hour &&
        now->minutes == alarm_minute && now->seconds < alarm_duration_seconds;

    if (!in_alarm_window) {
        if (alarm_sounding) stop_alarm();
        return;
    }

    if (!alarm_sounding) {
        alarm_sounding = true;
        melody_note = 0;
        start_alarm_note();
        return;
    }

    if (buzzer_tone_on && time_reached(melody_note_off_time)) {
        buzzer_stop_tone();
    }

    if (time_reached(melody_next_note_time)) {
        melody_note = (melody_note + 1) %
                      (sizeof(alarm_melody) / sizeof(alarm_melody[0]));
        start_alarm_note();
    }
}

/**
 * @brief Show the RTC time on the TM1637 display.
 */
void display_time(const ds3231_data_t* now) {
    uint8_t display_hour = now->hours;
    if (!twentyfour_hour) {
        display_hour %= 12;
        if (display_hour == 0) display_hour = 12;
    }

    TM1637_display_both(display_hour, now->minutes, true);
    TM1637_set_colon((now->seconds % 2) == 0);

    char date[sizeof("DD.MM.YYYY")];
    snprintf(date, sizeof(date), "%02u.%02u.%04u", now->date, now->month,
             2000u + (now->century ? 100u : 0u) + now->year);
    ssd1306_clear(&disp);
    ssd1306_draw_string(&disp, 8, 24, 2, date);
    ssd1306_show(&disp);
}

static unsigned int full_year(const ds3231_data_t* now) {
    return 2000u + (now->century ? 100u : 0u) + now->year;
}

/**
 * @brief Print the clock and alarm status to the configured Pico stdio UART.
 */
void print_time(const ds3231_data_t* now) {
    static const char* days[] = {"Monday", "Tuesday",  "Wednesday", "Thursday",
                                 "Friday", "Saturday", "Sunday"};
    const char* day_name =
        now->day >= 1 && now->day <= 7 ? days[now->day - 1] : "Unknown";

    printf("\n--- Status Report ---\n\n");
    printf("Current time: %04u/%02u/%02u (%s) %02u:%02u:%02u\n", full_year(now),
           now->month, now->date, day_name, now->hours, now->minutes,
           now->seconds);

    datetime_t datetime = {
        .year = (int16_t)full_year(now),
        .month = (int8_t)now->month,
        .day = (int8_t)now->date,
        .dotw = (int8_t)(now->day % 7),
        .hour = (int8_t)now->hours,
        .min = (int8_t)now->minutes,
        .sec = (int8_t)now->seconds,
    };
    time_t unix_time;
    if (datetime_to_time(&datetime, &unix_time)) {
        printf("UNIX time: %llds = %lldd\n", (long long)unix_time,
               (long long)unix_time / 86400LL);
    }

    uint8_t shown_alarm_hour = alarm_hour;
    const char* meridiem = "";
    if (!twentyfour_hour) {
        meridiem = alarm_hour < 12 ? " AM" : " PM";
        shown_alarm_hour %= 12;
        if (shown_alarm_hour == 0) shown_alarm_hour = 12;
    }

    printf("\nAlarm set for: %02u:%02u%s; Alarm is %s\n", shown_alarm_hour,
           alarm_minute, meridiem, alarm_enabled ? "enabled" : "disabled");
    printf("Alarm duration: %u seconds\n", alarm_duration_seconds);
    printf("Time format: %s\n", twentyfour_hour ? "24-hour" : "12-hour");
    printf("--- --- ----- --- ---\n\n");
}

static bool is_leap_year(unsigned int year) {
    return (year % 4u == 0u && year % 100u != 0u) || year % 400u == 0u;
}

static uint8_t days_in_month(unsigned int year, uint8_t month) {
    static const uint8_t month_lengths[] = {31, 28, 31, 30, 31, 30,
                                            31, 31, 30, 31, 30, 31};
    if (month == 2 && is_leap_year(year)) return 29;
    return month >= 1 && month <= 12 ? month_lengths[month - 1] : 0;
}

static uint8_t day_of_week(unsigned int year, uint8_t month, uint8_t date) {
    static const uint8_t offsets[] = {0, 3, 2, 5, 0, 3, 5, 1, 4, 6, 2, 4};
    unsigned int adjusted_year = year;
    if (month < 3) adjusted_year--;
    const unsigned int sunday_based =
        (adjusted_year + adjusted_year / 4u - adjusted_year / 100u +
         adjusted_year / 400u + offsets[month - 1] + date) %
        7u;
    return sunday_based == 0 ? 7 : (uint8_t)sunday_based;
}

static bool configure_rtc(unsigned int year, uint8_t month, uint8_t date,
                          uint8_t hour, uint8_t minute, uint8_t second) {
    if (year < 2000 || year > 2199 || month < 1 || month > 12 || date < 1 ||
        date > days_in_month(year, month) || hour > 23 || minute > 59 ||
        second > 59) {
        return false;
    }

    ds3231_data_t value = {
        .seconds = second,
        .minutes = minute,
        .hours = hour,
        .am_pm = false,
        .day = day_of_week(year, month, date),
        .date = date,
        .month = month,
        .century = (uint8_t)(year >= 2100),
        .year = (uint8_t)(year % 100),
    };

    if (ds3231_configure_time(&rtc, &value) != 0) return false;

    uint8_t status_register = DS3231_CONTROL_STATUS_REG;
    uint8_t status;
    if (i2c_write_blocking(I2C_PORT, I2C_DS3231_ADDRESS, &status_register, 1,
                           true) != 1 ||
        i2c_read_blocking(I2C_PORT, I2C_DS3231_ADDRESS, &status, 1, false) !=
            1) {
        return false;
    }
    status &= (uint8_t)~0x80u;
    const uint8_t clear_oscillator_stop[] = {DS3231_CONTROL_STATUS_REG, status};
    return i2c_write_blocking(I2C_PORT, I2C_DS3231_ADDRESS,
                              clear_oscillator_stop,
                              sizeof(clear_oscillator_stop),
                              false) == (int)sizeof(clear_oscillator_stop);
}

/**
 * @brief Initialize an untrusted RTC from the firmware build timestamp.
 */
bool set_rtc_to_build_time() {
    static const char* month_names = "JanFebMarAprMayJunJulAugSepOctNovDec";
    char month_name[4] = {};
    unsigned int year;
    unsigned int date;
    unsigned int hour;
    unsigned int minute;
    unsigned int second;

    if (sscanf(__DATE__, "%3s %u %u", month_name, &date, &year) != 3 ||
        sscanf(__TIME__, "%u:%u:%u", &hour, &minute, &second) != 3) {
        return false;
    }

    const char* month_position = strstr(month_names, month_name);
    if (month_position == nullptr) return false;
    const uint8_t month = (uint8_t)((month_position - month_names) / 3 + 1);
    return configure_rtc(year, month, (uint8_t)date, (uint8_t)hour,
                         (uint8_t)minute, (uint8_t)second);
}

static bool parse_integer(const char* line, int minimum, int maximum,
                          int* value) {
    int parsed;
    char trailing;
    if (sscanf(line, " %d %c", &parsed, &trailing) != 1 || parsed < minimum ||
        parsed > maximum) {
        return false;
    }
    *value = parsed;
    return true;
}

static void show_settings_menu() {
    printf("\n--- Settings Menu ---\n");
    printf("General settings\n");
    printf("s: Set time\n");
    printf("h: Set 12/24-hour format\n");
    printf("Alarm settings\n");
    printf("a: Set alarm time\n");
    printf("d: Set alarm duration\n");
    printf("e: Enable/disable alarm\n");
    printf("x: Exit settings\n");
    printf("Enter a command: ");
}

static void finish_setting() {
    serial_ui_state = SERIAL_UI_IDLE;
    printf("Commands: 't' status, 's' settings\n");
}

static void handle_serial_line(const char* line) {
    int value;

    switch (serial_ui_state) {
        case SERIAL_UI_IDLE: {
            if (strcmp(line, "t") == 0) {
                ds3231_data_t now;
                if (ds3231_read_current_time(&rtc, &now) == 0)
                    print_time(&now);
                else
                    printf("Could not read DS3231\n");
            } else if (strcmp(line, "s") == 0) {
                serial_ui_state = SERIAL_UI_MENU;
                show_settings_menu();
            } else if (line[0] != '\0') {
                printf("Unknown command. Use 't' or 's'.\n");
            }
            break;
        }

        case SERIAL_UI_MENU:
            if (strcmp(line, "s") == 0) {
                pending_time = {};
                serial_ui_state = SERIAL_UI_SET_YEAR;
                printf("Enter the year (2000-2199): ");
            } else if (strcmp(line, "h") == 0) {
                serial_ui_state = SERIAL_UI_SET_FORMAT;
                printf("Enter the time format (0 = 12-hour, 1 = 24-hour): ");
            } else if (strcmp(line, "a") == 0) {
                serial_ui_state = SERIAL_UI_SET_ALARM;
                printf("Enter the alarm time (24-hour HHMM or HH:MM): ");
            } else if (strcmp(line, "d") == 0) {
                serial_ui_state = SERIAL_UI_SET_DURATION;
                printf("Enter the alarm duration (1-60 seconds): ");
            } else if (strcmp(line, "e") == 0) {
                serial_ui_state = SERIAL_UI_SET_ENABLED;
                printf("Enter 1 to enable the alarm, 0 to disable: ");
            } else if (strcmp(line, "x") == 0) {
                finish_setting();
            } else {
                printf("Invalid settings command.\n");
                show_settings_menu();
            }
            break;

        case SERIAL_UI_SET_YEAR:
            if (parse_integer(line, 2000, 2199, &value)) {
                pending_time.century = (uint8_t)(value >= 2100);
                pending_time.year = (uint8_t)(value % 100);
                serial_ui_state = SERIAL_UI_SET_MONTH;
                printf("Enter the month (1-12): ");
            } else {
                printf("Invalid year. Enter 2000-2199: ");
            }
            break;

        case SERIAL_UI_SET_MONTH:
            if (parse_integer(line, 1, 12, &value)) {
                pending_time.month = (uint8_t)value;
                serial_ui_state = SERIAL_UI_SET_DATE;
                printf("Enter the day of month (1-31): ");
            } else {
                printf("Invalid month. Enter 1-12: ");
            }
            break;

        case SERIAL_UI_SET_DATE:
            if (parse_integer(line, 1, 31, &value)) {
                pending_time.date = (uint8_t)value;
                serial_ui_state = SERIAL_UI_SET_HOUR;
                printf("Enter the hour (0-23): ");
            } else {
                printf("Invalid day. Enter 1-31: ");
            }
            break;

        case SERIAL_UI_SET_HOUR:
            if (parse_integer(line, 0, 23, &value)) {
                pending_time.hours = (uint8_t)value;
                serial_ui_state = SERIAL_UI_SET_MINUTE;
                printf("Enter the minute (0-59): ");
            } else {
                printf("Invalid hour. Enter 0-23: ");
            }
            break;

        case SERIAL_UI_SET_MINUTE:
            if (parse_integer(line, 0, 59, &value)) {
                pending_time.minutes = (uint8_t)value;
                serial_ui_state = SERIAL_UI_SET_SECOND;
                printf("Enter the second (0-59): ");
            } else {
                printf("Invalid minute. Enter 0-59: ");
            }
            break;

        case SERIAL_UI_SET_SECOND:
            if (!parse_integer(line, 0, 59, &value)) {
                printf("Invalid second. Enter 0-59: ");
                break;
            }
            pending_time.seconds = (uint8_t)value;
            value = 2000 + (pending_time.century ? 100 : 0) + pending_time.year;
            if (configure_rtc((unsigned int)value, pending_time.month,
                              pending_time.date, pending_time.hours,
                              pending_time.minutes, pending_time.seconds)) {
                printf("Time set\n");
                finish_setting();
            } else {
                printf("Invalid date or DS3231 write failed. Start again.\n");
                finish_setting();
            }
            break;

        case SERIAL_UI_SET_FORMAT:
            if (strcmp(line, "0") == 0 || strcmp(line, "1") == 0) {
                twentyfour_hour = line[0] == '1';
                printf("Time format set to %s\n",
                       twentyfour_hour ? "24-hour" : "12-hour");
                finish_setting();
            } else {
                printf("Invalid format. Enter 0 or 1: ");
            }
            break;

        case SERIAL_UI_SET_ALARM: {
            int hour;
            int minute;
            char trailing;
            const bool parsed =
                (sscanf(line, " %2d:%2d %c", &hour, &minute, &trailing) == 2) ||
                (strlen(line) == 4 &&
                 sscanf(line, "%2d%2d", &hour, &minute) == 2);
            if (parsed && hour >= 0 && hour <= 23 && minute >= 0 &&
                minute <= 59) {
                alarm_hour = (uint8_t)hour;
                alarm_minute = (uint8_t)minute;
                printf("Alarm set to %02u:%02u\n", alarm_hour, alarm_minute);
                finish_setting();
            } else {
                printf("Invalid alarm time. Enter HHMM or HH:MM: ");
            }
            break;
        }

        case SERIAL_UI_SET_DURATION:
            if (parse_integer(line, 1, 60, &value)) {
                alarm_duration_seconds = (uint8_t)value;
                printf("Alarm duration set to %u seconds\n",
                       alarm_duration_seconds);
                finish_setting();
            } else {
                printf("Invalid duration. Enter 1-60: ");
            }
            break;

        case SERIAL_UI_SET_ENABLED:
            if (strcmp(line, "0") == 0 || strcmp(line, "1") == 0) {
                alarm_enabled = line[0] == '1';
                if (!alarm_enabled && alarm_sounding) stop_alarm();
                printf("Alarm %s\n", alarm_enabled ? "enabled" : "disabled");
                finish_setting();
            } else {
                printf("Invalid value. Enter 0 or 1: ");
            }
            break;
    }
}

/**
 * @brief Poll UART input and dispatch complete lines to the serial UI.
 */
void serial_ui_poll() {
    int input;
    while ((input = getchar_timeout_us(0)) != PICO_ERROR_TIMEOUT) {
        const char character = (char)input;
        if (character == '\r' || character == '\n') {
            if (serial_line_length == 0) continue;
            serial_line[serial_line_length] = '\0';
            handle_serial_line(serial_line);
            serial_line_length = 0;
        } else if ((character == '\b' || character == 127) &&
                   serial_line_length > 0) {
            serial_line_length--;
        } else if (character >= 32 && character <= 126 &&
                   serial_line_length < sizeof(serial_line) - 1) {
            serial_line[serial_line_length++] = character;
        }
    }
}
