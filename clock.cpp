#include <stdio.h>
#include <stdlib.h>

#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"

extern "C" {
#include <PicoTM1637.h>
}

#define CLOCK_FIRMWARE_VERSION "0.0.1"

#define SEVEN_SEG_CLK_PIN 2
#define SEVEN_SEG_DIO_PIN 3

void demo_tm1637();
int pico_led_init(void);
void pico_set_led(bool);
int pico_tm1637_init(void);

/**
 * @brief Main entry point for the program.
 *
 * @return int status code (0 for success, non-zero for failure)
 */
int main() {
    stdio_init_all();
    hard_assert(pico_led_init() == PICO_OK);
    hard_assert(pico_tm1637_init() == PICO_OK);

    for (int i = 0; i < 5; i++) {
        pico_set_led(true);
        sleep_ms(100);
        pico_set_led(false);
        sleep_ms(100);
    }

    printf("Hello, world!\n");

    char title[] = "CLK";
    TM1637_display_word(title, true);
    printf("CLK firmware version: %s\n", CLOCK_FIRMWARE_VERSION);

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
