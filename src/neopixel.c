#include "neopixel.h"

#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>

#include <zephyr/drivers/gpio.h>

#include <stdlib.h>


static uint8_t mPIXEL_DATA_PIN;
static neopixel_strip_t m_strip;

#define NEOPIXEL_NODE DT_NODELABEL(led1)
static const struct gpio_dt_spec led_neopixel = GPIO_DT_SPEC_GET(NEOPIXEL_NODE, gpios);

#define HAL_nsDelay(nano_sec) k_sleep(K_NSEC(nano_sec))
#define HAL_usDelay(micro_sec) k_sleep(K_USEC(micro_sec))
#define HAL_msDelay(mili_sec) k_sleep(K_MSEC(mili_sec))

// const struct device *neopixel_port;
gpio_port_pins_t neopixel_pin;

// NOTE 1 CLOCK CYCLE IS 7.8125ns


#define NEOPIXEL_SEND_ONE \
gpio_port_set_bits_raw(led_neopixel.port, neopixel_pin); \
__ASM ( \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
); \
gpio_port_clear_bits_raw(led_neopixel.port, neopixel_pin); \
__ASM ( \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
); //Logic "1" 52x1 + 50x0 * 15.625ns = 812.5ns H + 781.25ns L


#define NEOPIXEL_SEND_ZERO \
gpio_port_set_bits_raw(led_neopixel.port, neopixel_pin); \
__ASM ( \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
); \
gpio_port_clear_bits_raw(led_neopixel.port, neopixel_pin); \
__ASM ( \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
); //Logic "0" 19x1 + 72x0 * 15.625ns = 296.875ns H + 1125ns L




// #define NEOPIXEL_SEND_ONE \
// gpio_port_set_bits_raw(led_neopixel.port, neopixel_pin); \
// __ASM ( \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t" \
// ); \
// gpio_port_clear_bits_raw(led_neopixel.port, neopixel_pin); \
// __ASM ( \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// ); //Logic "1" 52x1 + 50x0 * 15.625ns = 812.5ns H + 781.25ns L


// #define NEOPIXEL_SEND_ZERO \
// gpio_port_set_bits_raw(led_neopixel.port, neopixel_pin); \
// gpio_port_clear_bits_raw(led_neopixel.port, neopixel_pin); \
// __ASM ( \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// " NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
// ); //Logic "0" 19x1 + 72x0 * 15.625ns = 296.875ns H + 1125ns L




#define NEOPIXEL_SET_ONE(void)		gpio_port_set_bits_raw(led_neopixel.port, (gpio_port_pins_t)BIT(led_neopixel.pin))

#define NEOPIXEL_SET_ZERO(void) 	gpio_port_clear_bits_raw(led_neopixel.port, (gpio_port_pins_t)BIT(led_neopixel.pin))


void neopixel_init(uint16_t num_leds)
{
	if (!device_is_ready(led_neopixel.port)) {
		printk("RGB LED port ERROR\n");
	}else{
		printk("RGB LED port OK\n");
	}

	if (!gpio_pin_configure_dt(&led_neopixel, GPIO_OUTPUT_INACTIVE)) {
		printk("RGB LED config LOW: OK\n");
	}else{
		printk("RGB LED config ERROR\n");
	}
	if (!gpio_pin_configure_dt(&led_neopixel, GPIO_OUTPUT_ACTIVE)) {
		printk("RGB LED config HIGH: OK\n");
	}else{
		printk("RGB LED config ERROR\n");
	}

    neopixel_pin = (gpio_port_pins_t)BIT(led_neopixel.pin);

	// mPIXEL_DATA_PIN=led_neopixel.pin;
	// m_strip.leds = (color_t*) malloc(sizeof(color_t) * num_leds);
	// m_strip.pin_num = mPIXEL_DATA_PIN;
	// m_strip.num_leds = num_leds;

	// for (int i = 0; i < num_leds; i++)
	// {	
	// 	m_strip.leds[i].simple.g = 0;
	// 	m_strip.leds[i].simple.r = 0;
	// 	m_strip.leds[i].simple.b = 0;
	// }
}


void neopixel_show_color(uint8_t r1, uint8_t g1, uint8_t b1, uint8_t r2, uint8_t g2, uint8_t b2, int msecs) {
    /**
     * Send this command to change the color of the LEDs.
     * 
     * Set to all 0's to turn the LEDs off.
     * 
     * THIS IS THE PRIMARY FUNCTION WE SHOULD BE USING.
     */

    // NEOPIXEL_SET_ZERO();
	// HAL_usDelay(500);

    // LED 1 Green
    if ((g1 & 128) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 64) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 32) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 16) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 8) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 4) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 2) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g1 & 1) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}

    // LED 1 Red
    if ((r1 & 128) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 64) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 32) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 16) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 8) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 4) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 2) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r1 & 1) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}

    // LED 1 Blue
    if ((b1 & 128) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 64) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 32) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 16) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 8) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 4) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 2) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b1 & 1) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}




    // LED 2 Green
    if ((g2 & 128) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 64) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 32) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 16) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 8) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 4) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 2) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((g2 & 1) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}

    // LED 2 Red
    if ((r2 & 128) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 64) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 32) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 16) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 8) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 4) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 2) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((r2 & 1) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}

    // LED 2 Blue
    if ((b2 & 128) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 64) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 32) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 16) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 8) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 4) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 2) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}
    if ((b2 & 1) > 0)	{NEOPIXEL_SEND_ONE}
    else	{NEOPIXEL_SEND_ZERO}

    for (int i=0; i<100; i++) {
        NEOPIXEL_SET_ZERO();
    }
    
    HAL_msDelay(msecs);

	// HAL_usDelay(50);
	//	nrf_gpio_pin_set(PIN);
}


// void neopixel_show()
// {
// 	//	const uint8_t PIN =  m_strip.pin_num;

// 	//	NRF_GPIO->OUTCLR = (1UL << PIN);
// 	//	nrf_gpio_pin_clear(PIN);
// 	//	nrf_gpio_pin_set(PIN);

// 	NEOPIXEL_SET_ZERO();
// 	HAL_usDelay(80);
// 	__disable_irq();
// 	for (int i = 0; i < m_strip.num_leds; i++)
// 	{
// 		//Serial.print(" Red: ");Serial.print(m_strip.leds[i].grb[0]);Serial.print(" Green: ");Serial.print(m_strip.leds[i].grb[1]);Serial.print(" Blue: ");Serial.println(m_strip.leds[i].grb[2]);
// 		for (int j = 0; j < 3; j++)
// 		{
// 			if ((m_strip.leds[i].grb[j] & 128) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 64) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 32) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 16) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 8) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 4) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 2) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
			
// 			if ((m_strip.leds[i].grb[j] & 1) > 0)	{NEOPIXEL_SEND_ONE}
// 			else	{NEOPIXEL_SEND_ZERO}
// 		}
// 	}

// 	HAL_usDelay(50);
// 	//	nrf_gpio_pin_set(PIN);
// 	__enable_irq();
// }

// void neopixel_set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue )
// {
// 	m_strip.leds[index].simple.r = red;
// 	m_strip.leds[index].simple.g = green;
// 	m_strip.leds[index].simple.b = blue;
// }

// void neopixel_set_color_and_show(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
// {
// 	m_strip.leds[index].simple.r = red;
// 	m_strip.leds[index].simple.g = green;
// 	m_strip.leds[index].simple.b = blue;
// 	neopixel_show();
// }

// void neopixel_clear()
// {
// 	for (int i = 0; i < m_strip.num_leds; i++)
// 	{
// 		m_strip.leds[i].simple.g = 0;
// 		m_strip.leds[i].simple.r = 0;
// 		m_strip.leds[i].simple.b = 0;
// 	}
// 	neopixel_show();
// }

// void neopixel_destroy()
// {
// 	free(m_strip.leds);
// 	m_strip.num_leds = 0;
// 	m_strip.pin_num = 0;
// }