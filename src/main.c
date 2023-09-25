#include <nrfx_clock.h>
#include <zephyr/zephyr.h>
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/sys/printk.h>
#include <zephyr/init.h>

#include <zephyr/types.h>
#include <stddef.h>
#include <errno.h>
#include <zephyr/sys/byteorder.h>

#include <zephyr/drivers/sensor.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/drivers/adc.h>

#include <stdio.h>
#include <zephyr/sys/__assert.h>

#include <stdio.h>
#include <stdlib.h>

#include "neopixel.h"

#define HAL_delay(ms_delay) k_sleep(K_MSEC(ms_delay))

/**
 * REMEMBER TO MODIFY CMAKELISTS TO INCLUDE NEOPIXEL.C!
 * 
 * REMEMBER TO MODIFY OVERLAY FILE TO SWITCH LED PIN
 */

void main(void)
{
	// Set 128 MHz clock speed
	// This is necessary so that the LED's can be controlled in time
	// Edit 9/24/23: actually this is not needed jk
	// nrfx_clock_divider_set(NRF_CLOCK_DOMAIN_HFCLK, NRF_CLOCK_HFCLK_DIV_1);

	k_msleep(500);

	printk("STARTING PROGRAM\n");

	int ret;

	// Note: this init might not be necessary
	neopixel_init(2);

	// neopixel_set_color(0, 255, 0, 0);
	// neopixel_set_color(1, 0, 255, 0);
    
	// neopixel_show();
	// k_sleep(K_MSEC(1000));

	// neopixel_set_color(0, 0, 0, 0);
	// neopixel_show();
	// HAL_delay(3000);

	// neopixel_set_color(0, 0, 0, 255);
	// neopixel_show();
	// HAL_delay(3000);

	test();


	neopixel_clear();

	neopixel_destroy();

	int ctr = 0;
	while(1){
		printk("looping...  %d\n", ctr);
		ctr++;

		// neopixel_set_color(0, 255, 0, 0);
		// neopixel_set_color(1, 255, 0, 0);
		// neopixel_show();
		// HAL_delay(1000);

		// neopixel_set_color(0, 0, 255, 0);
		// neopixel_set_color(1, 0, 255, 0);
		// neopixel_show();
		// HAL_delay(1000);

		// neopixel_set_color(0, 0, 0, 255);
		// neopixel_set_color(1, 0, 0, 255);
		// neopixel_show();
		// HAL_delay(1000);

		// neopixel_set_color(0, 255, 255, 255);
		// neopixel_set_color(1, 0, 0, 0);
		// neopixel_show();
		// HAL_delay(200);

		// neopixel_set_color(0, 0, 0, 0);
		// neopixel_set_color(1, 255, 255, 255);
		// neopixel_show();
		// HAL_delay(200);

		// neopixel_set_color(0, 0, 0, 0);
		// neopixel_set_color(1, 0, 0, 0);
		// neopixel_show();
		// HAL_delay(200);

		// neopixel_set_color(0, 0, 0, 0);
		// neopixel_set_color(1, 0, 0, 0);
		// neopixel_show();
		// HAL_delay(200);

		// neopixel_set_color(0, 0, 0, 0);
		// neopixel_set_color(1, 0, 0, 0);
		// neopixel_show();
		// HAL_delay(200);

		// neopixel_set_color(0, 255, 0, 0);
		// neopixel_set_color(1, 0, 255, 0);

		// neopixel_show();
		k_sleep(K_MSEC(1000));
	}
}