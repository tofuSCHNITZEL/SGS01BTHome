/********************************************************************************************************
 * @file    app_battery.c
 *
 * @brief   Battery voltage ADC
 *
 * @author  haraldapp
 * @date    08,2026
 *
 * @par     Copyright (c) 2024-2026, haraldapp, https://github.com/haraldapp
 *
 *          Licensed under the Apache License, Version 2.0 (the "License");
 *          you may not use this file except in compliance with the License.
 *          You may obtain a copy of the License at
 *              http://www.apache.org/licenses/LICENSE-2.0
 *          Unless required by applicable law or agreed to in writing, software
 *          distributed under the License is distributed on an "AS IS" BASIS,
 *          WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *          See the License for the specific language governing permissions and
 *          limitations under the License.
 *******************************************************************************************************/

#include "app_config.h"

#if (APP_BATTERY_CHECK)  // component enabled
#include "tl_common.h"
#include "drivers.h"

#include "app.h"
#include "types.h"
#include "gpio.h"
#include "adc.h"
#include "pm.h"

#ifndef APP_BATTERY_CHECK_LOG_EN
#define APP_BATTERY_CHECK_LOG_EN 0
#endif
#ifndef APP_BATTERY_CHECK_DEBUG_EN
#define APP_BATTERY_CHECK_DEBUG_EN 0
#endif

#ifndef APP_SUPPLYVOLTAGE_CRITICAL_MV
#define APP_SUPPLYVOLTAGE_CRITICAL_MV	2200 // critical battery voltage mV
#endif

#ifndef APP_SUPPLYVOLTAGE_CRITICAL_THRESHOLD
#define APP_SUPPLYVOLTAGE_CRITICAL_THRESHOLD 200 // mV
#endif

#ifndef APP_CONFIG_BATTERY_PIN_DIVIDER_R1
#define APP_CONFIG_BATTERY_PIN_DIVIDER_R1 750
#define APP_CONFIG_BATTERY_PIN_DIVIDER_R2 330
#endif

_attribute_data_retention_	u32	app_battery_check_time_sec = 0;
_attribute_data_retention_	u32	app_battery_fail_delay_sec = 0;
_attribute_data_retention_	u8	app_battery_check_next = 0;
_attribute_data_retention_	u8	app_battery_check_batpin = 0;

//
// reading chip supply voltage
// reading battery voltage (pin connected to a resistor divider)
//


_attribute_data_retention_ volatile u8 batpin_supply_hw_initialized = 0;
_attribute_data_retention_ volatile u8 batpin_battery_hw_initialized = 0;
_attribute_data_retention_ volatile u16 batpin_gpio = APP_BATTERY_BATPIN_None;
#define BATPIN_SAMPLE_CNT 8
_attribute_data_retention_	volatile u32 batpin_buf[BATPIN_SAMPLE_CNT];

extern unsigned short adc_gpio_calib_vref; // from adc.c
extern signed char adc_gpio_calib_vref_offset; // from adc.c
extern unsigned char adc_pre_scale; // from adc.c

static inline int calibrate_adc_value(int val)
{
	return ((val*adc_pre_scale*adc_gpio_calib_vref)>>13) + adc_gpio_calib_vref_offset; // calibration from adc.c
}

static u8 gpio_to_pchannel(u16 pin)
{
	static const struct { u16 pin; u8 chn; } _g2p[]={
		{GPIO_PB0,B0P},	{GPIO_PB1,B1P}, {GPIO_PB2,B2P},	{GPIO_PB3,B3P},
		{GPIO_PB4,B4P},	{GPIO_PB5,B5P}, {GPIO_PB6,B6P},	{GPIO_PB7,B7P},
		{GPIO_PC4,C4P}, {GPIO_PC5,C5P}
	};
	for (u8 u=0; u<sizeof(_g2p)/sizeof(_g2p[0]); u++)
		if (_g2p[u].pin == pin)
			return _g2p[u].chn;
	return NOINPUTP;
}

// batpin_gpio
_attribute_ram_code_ void batpin_adc_init(u16 pin)
{
	if (pin == APP_BATTERY_BATPIN_None)   return;
	adc_power_on_sar_adc(0); // power off SAR ADC
	gpio_set_input_en(pin, 1);
	gpio_setup_up_down_resistor(pin, PM_PIN_UP_DOWN_FLOAT);
	adc_set_sample_clk(5); // ADC sample 4MHz
	adc_set_left_right_gain_bias(GAIN_STAGE_BIAS_PER100, GAIN_STAGE_BIAS_PER100); // set ADC L R channel Gain Stage bias
	adc_set_chn_enable_and_max_state_cnt(ADC_MISC_CHN, 2); // set total length for sampling state machine and channel
	adc_set_state_length(240, 0, 10);  	// set R_max_mc,R_max_c,R_max_s (240 = 10.4ms)
	analog_write (anareg_adc_res_m, RES14 | FLD_ADC_EN_DIFF_CHN_M); // resolution 14 bit,  differential mode
	adc_set_ain_chn_misc(gpio_to_pchannel(pin), GND);
	adc_set_ref_voltage(ADC_MISC_CHN, ADC_VREF_1P2V); // vref 1.2V
	adc_set_tsample_cycle_chn_misc(SAMPLING_CYCLES_6); // 6 cycles sampling
	adc_set_ain_pre_scaler(ADC_PRESCALER_1F8); // prescaler 1/8
	adc_power_on_sar_adc(1); // power on SAR ADC
}

_attribute_ram_code_ u32 batpin_read_voltage(u16 pin, u16 div_r1, u16 div_r2)
{
	DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] ADC read pin 0x%X", pin);
	adc_reset_adc_module();
	s8 i; u32 tstart = clock_time();
	for (i=0; i<BATPIN_SAMPLE_CNT; i++)  batpin_buf[i] = 0;
	while (!clock_time_exceed(tstart, 25));  // 2 sample cycle delay
	adc_config_misc_channel_buf((u16 *)batpin_buf, BATPIN_SAMPLE_CNT*4); // reinit (lost in suspend)
	dfifo_enable_dfifo2();
	s8 min_idx=-1, max_idx=-1; int min_val=0xFFFF, max_val=0;
	for (i=0; i<BATPIN_SAMPLE_CNT; i++)
	{
		while(!batpin_buf[i]); // wait for ADC complete
		u32 val=batpin_buf[i];
		if (val & BIT(13))   val=0; // 14 bits data, bit 13 is sign bit
		else                 val&=0x1FFF;  //BIT(12..0) is valid adc result
		#if (BATPIN_SAMPLE_CNT>4)
		if (min_idx<0 || val<min_val)  { min_idx=i; min_val=val; }
		if (max_idx<0 || val>max_val)  { max_idx=i; max_val=val; }
		#endif
		batpin_buf[i]=val;
	}
	dfifo_disable_dfifo2(); // disable adc fifo
	int val_avg=0; u8 val_avg_cnt=0;
	for (i=0; i<BATPIN_SAMPLE_CNT; i++)
	{
		u8 drop=0; if (i==min_idx || i==max_idx)   drop=1; // drop min/max values
		if (!drop)  { val_avg+=batpin_buf[i]; val_avg_cnt++; }
	}
	val_avg/=val_avg_cnt;
	val_avg = calibrate_adc_value(val_avg); // calibration from adc.c
	if (div_r1>0)  val_avg = (val_avg*(div_r1+div_r2))/div_r2;
	if (val_avg<0)   val_avg=0;
	#if (APP_BATTERY_CHECK_DEBUG_EN)
	for (i=0; i<BATPIN_SAMPLE_CNT; i++)
	{
		int v1=batpin_buf[i]; const char *d=(i==min_idx || i==max_idx)?"drop":"";
		int v2=calibrate_adc_value(v1);
		int v3=v2; if (div_r1>0)  v3 = (v3*(div_r1+div_r2))/div_r2;
		DEBUGFMT(1, "[BAT] %d - adc:%d calib:%d div:%d %s", i, v1, v2, v3, d);
	}
	DEBUGFMT(1, "[BAT] average:%d", val_avg);
	#endif
	return (u32)val_avg;
}

_attribute_ram_code_ u32 batpin_read_supply_voltage()
{
	if(!batpin_supply_hw_initialized)
	{
		DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] Init ADC supply voltage (dummy pin 0x%X)", APP_SUPPLYVOLTAGE_PIN);
		batpin_adc_init(APP_SUPPLYVOLTAGE_PIN);
		gpio_set_output_en(APP_SUPPLYVOLTAGE_PIN, 1);
		gpio_write(APP_SUPPLYVOLTAGE_PIN, 1);
		batpin_supply_hw_initialized = 1; batpin_battery_hw_initialized = 0;
	}
	return batpin_read_voltage(APP_SUPPLYVOLTAGE_PIN, 0, 0);
}

_attribute_ram_code_ u32 batpin_read_battery_voltage()
{
	DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] Read battery voltage (initialized=%u)", batpin_battery_hw_initialized);
	if (batpin_gpio == APP_BATTERY_BATPIN_None)
		return 0;
	if (batpin_gpio == APP_BATTERY_BATPIN_Supply)
	{
		return batpin_read_supply_voltage();
	}
	if(!batpin_battery_hw_initialized)
	{
		DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] Init ADC batpin 0x%X", batpin_gpio);
		batpin_adc_init(batpin_gpio);
		gpio_set_output_en(batpin_gpio, 0);
		batpin_battery_hw_initialized = 1; batpin_supply_hw_initialized = 0;
	}
	return batpin_read_voltage(batpin_gpio, APP_CONFIG_BATTERY_PIN_DIVIDER_R1, APP_CONFIG_BATTERY_PIN_DIVIDER_R2);
}

u8 batpin_test_battery_pin_connected()
{
	if (batpin_gpio == APP_BATTERY_BATPIN_None)   return 0;
	batpin_adc_init(batpin_gpio);
	gpio_setup_up_down_resistor(batpin_gpio, PM_PIN_PULLUP_1M);
	u32 batpin_mv=batpin_read_voltage(batpin_gpio, 0, 0);
	u8 ok=(batpin_mv>1000 && batpin_mv<2100)?1:0;
	batpin_battery_hw_initialized = 1; batpin_supply_hw_initialized = 0;
	gpio_setup_up_down_resistor(batpin_gpio, PM_PIN_UP_DOWN_FLOAT);
	DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] batpin check voltage: %u %s", batpin_mv, ok?"ok":"fail" );
    /* for adjusting/develop/debug
	enum { PULLUP_DIVIDER_R=1000 }; // 1M
	enum { MAX_BAT_VOLTAGE_MV=4200 }; // 6V
	enum { CALCFACT=1000 }; // factor to calculate in a good integer range
	u32 supply_mv=batpin_read_supply_voltage();
	batpin_adc_init(batpin_gpio);
	gpio_setup_up_down_resistor(batpin_gpio, PM_PIN_PULLUP_1M);
	u32 batpin_mv=batpin_read_voltage(batpin_gpio, 0, 0);
	DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] batpin check voltage: %u", batpin_mv );
    u8 ok=(batpin_mv>1000 && batpin_mv<2100);
	gpio_setup_up_down_resistor(batpin_gpio, PM_PIN_UP_DOWN_FLOAT);
	u32 i1=(supply_mv*CALCFACT)/(PULLUP_DIVIDER_R+APP_CONFIG_BATTERY_PIN_DIVIDER_R2); // only "about" calculation
	u32 i2_min=(supply_mv*CALCFACT)/(APP_CONFIG_BATTERY_PIN_DIVIDER_R1+APP_CONFIG_BATTERY_PIN_DIVIDER_R2);
	u32 i2_max=(MAX_BAT_VOLTAGE_MV*CALCFACT)/(APP_CONFIG_BATTERY_PIN_DIVIDER_R1+APP_CONFIG_BATTERY_PIN_DIVIDER_R2);
	u32 u_min=((i1+i2_min)*APP_CONFIG_BATTERY_PIN_DIVIDER_R2)/CALCFACT; u_min=(u_min*8)/10;
	u32 u_max=((i1+i2_max)*APP_CONFIG_BATTERY_PIN_DIVIDER_R2)/CALCFACT;
    u8 ok=(batpin_mv>=u_min && batpin_mv<=u_max);
	DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] batpin check: v=%u min=%u max=%u: %s", batpin_mv, u_min, u_max, ok?"ok":"fail");
	batpin_battery_hw_initialized = 0; batpin_supply_hw_initialized = 0;
	*/
	return ok;
}


//
// App interface
//

static void notify_battery_voltage(u32 mv)
{
	volatile u16 notify_v=(u16)mv;
	app_notify(APP_NOTIFY_BATTERYVOLTAGE, (u8*)&notify_v, 2 );
}


void app_battery_init_normal(void) // battery_check.c
{
	DEBUGSTR(APP_BATTERY_CHECK_LOG_EN, "[BAT] Battery init");
	batpin_supply_hw_initialized = 0; batpin_battery_hw_initialized = 0;
	u16 check_mv=APP_SUPPLYVOLTAGE_CRITICAL_MV;
	u8 app_state=app_flash_get_persist_state();
	if (app_state & APP_STATE_LOWBAT)
	{
		check_mv+=APP_SUPPLYVOLTAGE_CRITICAL_THRESHOLD;
	}
	u32 supply_mv=batpin_read_supply_voltage();
	u8 bat_ok=(supply_mv>=check_mv)?1:0;
	if (bat_ok)
	{
		app_flash_set_persist_state(0, APP_STATE_LOWBAT); // reset low battery state
		app_battery_check_time_sec=app_sec_time(); // battery check interval
	}
	else
	{
		DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] The supply voltage is lower than %dmV - shut down", check_mv);
		app_flash_set_persist_state(APP_STATE_LOWBAT, APP_STATE_LOWBAT);
		cpu_sleep_wakeup(DEEPSLEEP_MODE, 0, 0);  // deepsleep
	}
}

_attribute_ram_code_ void app_battery_init_deepRetn(void)
{
	// ADC setting will be lost during deep sleep
	batpin_supply_hw_initialized = 0; batpin_battery_hw_initialized = 0;
}

u8 app_battery_loop(void)
{
	u8 low_bat_state;
	// running on low bat - delayed stop
	low_bat_state=(app_flash_get_persist_state()&APP_STATE_LOWBAT);
	if (low_bat_state && app_sec_time_exceeds(app_battery_fail_delay_sec,APP_BATTERY_FAIL_DELAY_SEC))
	{
		DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] Delayed shutdown (init:%u now:%u)", app_battery_fail_delay_sec, app_sec_time());
		DEBUGSTR(APP_BATTERY_CHECK_LOG_EN, "[BAT] Battery low: deep sleep");
		cpu_sleep_wakeup(DEEPSLEEP_MODE, 0 /*PM_WAKEUP_PAD*/, 0);  // deep sleep
	}
	// test once if battery pin is connected
	if (!low_bat_state && app_battery_check_batpin)
	{
		u8 ok=batpin_test_battery_pin_connected();
		if (!ok)
		{
			DEBUGSTR(APP_BATTERY_CHECK_LOG_EN, "[BAT] Check battery pin: not connected");
			batpin_gpio=APP_BATTERY_BATPIN_None;
		}
		app_battery_check_batpin=0;
	}
	// periodic battery check
	if (!low_bat_state && (app_battery_check_next || app_sec_time_exceeds(app_battery_check_time_sec,APP_BATTERY_CHECK_INTERVAL_SEC)))
	{
		u32 supply_mv=batpin_read_supply_voltage();
		u8 supply_ok=(supply_mv>=APP_SUPPLYVOLTAGE_CRITICAL_MV)?1:0;
		if (supply_ok)
		{
			DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] Measure supply voltage %u mV", supply_mv);
			if (batpin_gpio == APP_BATTERY_BATPIN_Supply)
			{
				notify_battery_voltage(supply_mv);
				if (supply_mv < APP_BATTERY_LOW_MV)
					app_notify(APP_NOTIFY_BATTERYLOW, 0, 0 );
			}
			if (batpin_gpio < APP_BATTERY_BATPIN_GPIO_max)
			{
				u32 bat_mv=batpin_read_battery_voltage();
				DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] Measure battery voltage %u mV", bat_mv);
				notify_battery_voltage(bat_mv);
				if (bat_mv < APP_BATTERY_LOW_MV)
					app_notify(APP_NOTIFY_BATTERYLOW, 0, 0 );
			}
			app_battery_check_time_sec=app_sec_time(); // next check time
		}
		else
		{
			DEBUGFMT(APP_BATTERY_CHECK_LOG_EN, "[BAT] The supply voltage is lower than %dmV - delayed shut down", APP_SUPPLYVOLTAGE_CRITICAL_MV);
			app_flash_set_persist_state(APP_STATE_LOWBAT, APP_STATE_LOWBAT);
			app_battery_fail_delay_sec=app_sec_time(); // start timer for delayed stop
		}
		app_battery_check_next = 0;
	}
	return APP_PM_DEFAULT;
}

void app_battery_check_delayed(void)
{
	app_battery_check_next = 1; // run check in next loop
}

void app_battery_set_batpin(u16 batpin)
{
	DEBUGFMT(APP_BATTERY_CHECK_DEBUG_EN, "[BAT] Set batpin 0x%X", batpin);
	batpin_gpio=batpin;
	if (batpin_gpio < APP_BATTERY_BATPIN_GPIO_max)  app_battery_check_batpin=1;
	app_battery_check_next=1;
}


#endif // #if (APP_BATTERY_CHECK)








