/**
 * @file clock.cpp
 * @author Benjamin Hartmann
 * @brief Main program for the Raspberry Pi Pico clock project.
 * @version 0.0.1
 * @date 2026-07-19
 *
 * @copyright Copyright (c) 2026 Benjamin Hartmann
 *
 * TODO:
 * - Add SSD1315 OLED display.
 * - Add DS3231 RTC module.
 * - add Buzzer for alarm.
 * - add rotary encoder for setting time and alarm.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "hardware/i2c.h"
#include "pico/binary_info.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

extern "C" {
#include <PicoTM1637.h>
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

ssd1306_t disp;

void demo_tm1637();
int pico_led_init();
void pico_set_led(bool);
int pico_tm1637_init();
int pico_i2c_init();
int pico_ssd1306_init();
void bus_scan();
void animation();

/**
 * @brief Main entry point for the program.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int main() {
    stdio_init_all();
    hard_assert(pico_led_init() == PICO_OK);
    hard_assert(pico_tm1637_init() == PICO_OK);
    hard_assert(pico_i2c_init() == PICO_OK);
    hard_assert(pico_ssd1306_init() == PICO_OK);

    for (int i = 0; i < 5; i++) {
        pico_set_led(true);
        sleep_ms(100);
        pico_set_led(false);
        sleep_ms(100);
    }

    char title[] = "CLK";
    TM1637_display_word(title, true);
    printf("CLK firmware version: %s\n", CLOCK_FIRMWARE_VERSION);

    bus_scan();
    animation();

    // void TM1637_display(int number, bool leadingZeros);
    // void TM1637_display_word(char *word, bool leftAlign);
    // void TM1637_display_left(int number, bool leadingZeros);
    // void TM1637_display_right(int number, bool leadingZeros);
    // void TM1637_display_both(int leftNumber, int rightNumber, bool
    // leadingZeros); void TM1637_set_colon(bool on); void
    // TM1637_set_brightness(int val); void TM1637_clear();
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

int pico_ssd1306_init() {
    ssd1306_init(&disp, I2C_WIDTH, I2C_HEIGHT, I2C_OLED_ADDRESS, I2C_PORT);
    ssd1306_clear(&disp);
    return PICO_OK;
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
 * @brief Demonstrate the functionality of the TM1637 display.
 */
void demo_tm1637() {
    char demo_word[] = "dEMO";
    TM1637_display_word(demo_word, true);
    sleep_ms(2000);

    TM1637_put_4_bytes(1, 0x4f5b06);  // raw bytes for 123
    sleep_ms(1000);
    TM1637_set_brightness(0);  // brightness is not updated automatically,
    TM1637_put_4_bytes(1, 0x4f5b06);  // something new needs to be displayed.
    sleep_ms(1000);

    TM1637_clear();
    sleep_ms(500);

    printf("DEMO\n");
    // Count down from 150 to -50
    int count = 150;
    TM1637_display(count, false);
    sleep_ms(500);
    while (count >= -50) {
        TM1637_display(count, false);
        count--;
        // The display can not update too often. So even though there is no
        // sleep, this will take a couple of moments.
    }

    sleep_ms(1000);
    TM1637_clear();
    sleep_ms(500);

    // Demo a clock, by default there will be a colon between the numbers.
    int seconds = 0;
    int minutes = 0;
    TM1637_display_both(minutes, seconds, true);
    while (true) {
        sleep_ms(1000);
        seconds++;
        if (seconds == 60) {
            seconds = 0;
            minutes++;
            TM1637_display_both(minutes, seconds, true);
        } else {
            TM1637_display_right(seconds, true);
        }
    }
}

/**
 * I2C reserves some addresses for special purposes. We exclude these from the
 * scan. These are any addresses of the form 000 0xxx or 111 1xxx
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
 * @brief Demonstrate the functionality of the SSD1306 display.
 */
void demo_ssd1306() {
    const char* words[] = {"SSD1306", "DISPLAY", "DRIVER"};
    int SLEEPTIME = 10;

    printf("ANIMATION!\n");

    char buf[8];

    while (1) {
        for (int y = 0; y < 31; ++y) {
            ssd1306_draw_line(&disp, 0, y, 127, y);
            ssd1306_show(&disp);
            sleep_ms(SLEEPTIME);
            ssd1306_clear(&disp);
        }

        for (int y = 0, i = 1; y >= 0; y += i) {
            ssd1306_draw_line(&disp, 0, 31 - y, 127, 31 + y);
            ssd1306_draw_line(&disp, 0, 31 + y, 127, 31 - y);
            ssd1306_show(&disp);
            sleep_ms(SLEEPTIME);
            ssd1306_clear(&disp);
            if (y == 32) i = -1;
        }

        for (int i = 0; i < sizeof(words) / sizeof(char*); ++i) {
            ssd1306_draw_string(&disp, 8, 24, 2, words[i]);
            ssd1306_show(&disp);
            sleep_ms(SLEEPTIME * 20);
            ssd1306_clear(&disp);
        }

        for (int y = 31; y < 63; ++y) {
            ssd1306_draw_line(&disp, 0, y, 127, y);
            ssd1306_show(&disp);
            sleep_ms(SLEEPTIME);
            ssd1306_clear(&disp);
        }

        ssd1306_show(&disp);
        sleep_ms(2000);
    }
}
