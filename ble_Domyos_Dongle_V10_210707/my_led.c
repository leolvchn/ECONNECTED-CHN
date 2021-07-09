
#include <stdio.h>
#include "boards.h"
#include "app_error.h"
#include "nrf_delay.h"
#include "nrf_drv_gpiote.h"
//#include "app_simple_timer.h"
#include "app_timer.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

#define P_LED_RED				21
#define P_LED_BLUE			18
#define P_LED_GREEN			17

#define LED_RED(s)			(s==0)?(nrf_drv_gpiote_out_set(P_LED_RED)):(nrf_drv_gpiote_out_clear(P_LED_RED))
#define LED_BLUE(s)			(s==0)?(nrf_drv_gpiote_out_set(P_LED_BLUE)):(nrf_drv_gpiote_out_clear(P_LED_BLUE))
#define LED_GREEN(s)			(s==0)?(nrf_drv_gpiote_out_set(P_LED_GREEN)):(nrf_drv_gpiote_out_clear(P_LED_GREEN))



typedef struct{
	uint16_t cnt;
	uint16_t on;
	uint16_t period;
	bool 			b_sw;
}my_led_single;

struct{
	my_led_single	r;
	my_led_single	g;
	my_led_single b;
}my_led;

APP_TIMER_DEF(m_led_timer_id);
static void my_led_timer_handler(void * p_context);

#define MLED_TIMER_INTERVAL     	APP_TIMER_TICKS(250) /**< Battery level measurement interval (ticks). */


void mLED_init(void)
{
	uint32_t err_code;
	
	nrf_drv_gpiote_out_config_t out_config = GPIOTE_CONFIG_OUT_SIMPLE(false);

//=====================gpio
	nrf_drv_gpiote_in_uninit(P_LED_RED);
	nrf_drv_gpiote_out_uninit(P_LED_BLUE);
	nrf_drv_gpiote_out_uninit(P_LED_GREEN);
	err_code = nrf_drv_gpiote_out_init(P_LED_RED, &out_config);
	NRF_LOG_INFO("r init:%d", err_code);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(P_LED_BLUE, &out_config);
	NRF_LOG_INFO("b init:%d", err_code);
	APP_ERROR_CHECK(err_code);
	err_code = nrf_drv_gpiote_out_init(P_LED_GREEN, &out_config);
	NRF_LOG_INFO("g init:%d", err_code);
	APP_ERROR_CHECK(err_code);
	
	LED_RED(0);
	LED_BLUE(0);
	LED_GREEN(0);
//===================== timer	
	err_code = app_timer_create(&m_led_timer_id,
                                APP_TIMER_MODE_REPEATED,
                                my_led_timer_handler);
//	NRF_LOG_INFO("app_timer_create:%d", err_code);
	APP_ERROR_CHECK(err_code);
	
	err_code = app_timer_start(m_led_timer_id, MLED_TIMER_INTERVAL, NULL);
//	NRF_LOG_INFO("app_timer_start:%d", err_code);
	APP_ERROR_CHECK(err_code);

	NRF_LOG_INFO("my led init");
	
	memset(&my_led, 0, sizeof(my_led));
}

static void my_led_timer_handler(void * p_context)
{
	UNUSED_PARAMETER(p_context);

	if(my_led.r.b_sw)
	{
		my_led.r.cnt++;
		if(my_led.r.cnt == my_led.r.on)
		{
			LED_RED(0);
		}
		else if(my_led.r.cnt >= my_led.r.period)
		{
			my_led.r.cnt = 0;
			LED_RED(1);
		}
	}
	
	if(my_led.g.b_sw)
	{
		my_led.g.cnt++;
		if(my_led.g.cnt == my_led.g.on)
		{
			LED_GREEN(0);
		}
		else if(my_led.g.cnt >= my_led.g.period)
		{
			my_led.g.cnt = 0;
			LED_GREEN(1);
		}
	}
	
	if(my_led.b.b_sw)
	{
		my_led.b.cnt++;
		if(my_led.b.cnt == my_led.b.on)
		{
			LED_BLUE(0);
		}
		else if(my_led.b.cnt >= my_led.b.period)
		{
			my_led.b.cnt = 0;
			LED_BLUE(1);
		}
	}
}

void mLED_RED(uint16_t on, uint16_t period)
{
	my_led.r.on = on / 25;
	my_led.r.period = period / 25;
	
	if(period)
	{
		if(my_led.r.b_sw == false)
		{
			my_led.r.cnt = my_led.r.period;
		}
		my_led.r.b_sw = true;
	}
	else
	{
		if(on)
			LED_RED(1);
		else
			LED_RED(0);
		
		my_led.r.b_sw = false;
	}
}

void mLED_GREEN(uint16_t on, uint16_t period)
{
	my_led.g.on = on / 25;
	my_led.g.period = period / 25;
	
	if(period)
	{
		if(my_led.g.b_sw == false)
		{
			my_led.g.cnt = my_led.g.period;
		}
		my_led.g.b_sw = true;
	}
	else
	{
		if(on)
			LED_GREEN(1);
		else
			LED_GREEN(0);
		
		my_led.g.b_sw = false;
	}
}

void mLED_BLUE(uint16_t on, uint16_t period)
{
	my_led.b.on = on / 25;
	my_led.b.period = period / 25;
	
	if(period)
	{
		if(my_led.b.b_sw == false)
		{
			my_led.b.cnt = my_led.b.period;
		}
		my_led.b.b_sw = true;
	}
	else
	{
		if(on)
			LED_BLUE(1);
		else
			LED_BLUE(0);
		
		my_led.b.b_sw = false;
	}
}



