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
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t NOP\n\t" \
" NOP\n\t NOP\n\t" \
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
    // neopixel_port = &led_neopixel.port;

//	gpio_pin_set_dt(&led_neopixel, 1);
/*
	nrf_gpio_cfg_output(mPIXEL_DATA_PIN);
	NRF_GPIO->OUTCLR = (1UL << mPIXEL_DATA_PIN);
	nrf_gpio_pin_clear(mPIXEL_DATA_PIN);
	nrf_gpio_cfg_output(mPIXEL_DATA_PIN);
	NRF_GPIO->OUTCLR = (1UL << mPIXEL_DATA_PIN);
	nrf_gpio_pin_set(mPIXEL_DATA_PIN);
*/

	mPIXEL_DATA_PIN=led_neopixel.pin;
	m_strip.leds = (color_t*) malloc(sizeof(color_t) * num_leds);
	m_strip.pin_num = mPIXEL_DATA_PIN;
	m_strip.num_leds = num_leds;

	for (int i = 0; i < num_leds; i++)
	{	
		m_strip.leds[i].simple.g = 0;
		m_strip.leds[i].simple.r = 0;
		m_strip.leds[i].simple.b = 0;
	}

}

void neopixel_clear()
{
	for (int i = 0; i < m_strip.num_leds; i++)
	{
		m_strip.leds[i].simple.g = 0;
		m_strip.leds[i].simple.r = 0;
		m_strip.leds[i].simple.b = 0;
	}
	neopixel_show();
}

void test() {
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE

    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ONE
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO

    NEOPIXEL_SET_ZERO();

    HAL_msDelay(3000);
    
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO

    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO
    NEOPIXEL_SEND_ZERO

}

void neopixel_show()
{
	//	const uint8_t PIN =  m_strip.pin_num;

	//	NRF_GPIO->OUTCLR = (1UL << PIN);
	//	nrf_gpio_pin_clear(PIN);
	//	nrf_gpio_pin_set(PIN);

	NEOPIXEL_SET_ZERO();
	HAL_usDelay(80);
	__disable_irq();
	for (int i = 0; i < m_strip.num_leds; i++)
	{
		//Serial.print(" Red: ");Serial.print(m_strip.leds[i].grb[0]);Serial.print(" Green: ");Serial.print(m_strip.leds[i].grb[1]);Serial.print(" Blue: ");Serial.println(m_strip.leds[i].grb[2]);
		for (int j = 0; j < 3; j++)
		{
			if ((m_strip.leds[i].grb[j] & 128) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 64) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 32) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 16) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 8) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 4) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 2) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
			
			if ((m_strip.leds[i].grb[j] & 1) > 0)	{NEOPIXEL_SEND_ONE}
			else	{NEOPIXEL_SEND_ZERO}
		}
	}

	HAL_usDelay(50);
	//	nrf_gpio_pin_set(PIN);
	__enable_irq();
}

void neopixel_set_color(uint16_t index, uint8_t red, uint8_t green, uint8_t blue )
{
	m_strip.leds[index].simple.r = red;
	m_strip.leds[index].simple.g = green;
	m_strip.leds[index].simple.b = blue;
}

void neopixel_set_color_and_show(uint16_t index, uint8_t red, uint8_t green, uint8_t blue)
{
	m_strip.leds[index].simple.r = red;
	m_strip.leds[index].simple.g = green;
	m_strip.leds[index].simple.b = blue;
	neopixel_show();
}

void neopixel_destroy()
{
	free(m_strip.leds);
	m_strip.num_leds = 0;
	m_strip.pin_num = 0;
}