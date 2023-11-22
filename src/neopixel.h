#ifndef NEOPIXEL_H
#define NEOPIXEL_H


#include <stdbool.h>
#include <stdint.h>

#include <zephyr/drivers/gpio.h>

typedef union {
		struct {
			uint8_t g, r, b;
		}simple;
	uint8_t grb[3];
} color_t;
typedef struct {
	uint8_t pin_num;
	uint16_t num_leds;
	color_t *leds;
} neopixel_strip_t;

// extern const struct device *neopixel_port;
extern gpio_port_pins_t neopixel_pin;

void neopixel_init(uint16_t num_leds);
void neopixel_clear();
void neopixel_show();
void neopixel_show_color(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, int msecs);
void neopixel_set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue );
void neopixel_set_color_and_show(uint16_t index, uint8_t red, uint8_t green, uint8_t blue);
void neopixel_destroy();

#endif  // NEOPIXEL_H 