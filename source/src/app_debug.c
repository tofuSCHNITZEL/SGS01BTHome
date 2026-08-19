/********************************************************************************************************
 * @file    app_debug.c
 *
 * @brief   Debug helpers (using Telink SDK)
 *
 * @author  haraldapp
 * @date    06,2024
 *
 * @par     Copyright (c) 2024, haraldapp, https://github.com/haraldapp
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
#include "app.h"

#ifndef APP_DEBUG_ENABLE
#define APP_DEBUG_ENABLE 0
#endif
#ifndef APP_LOG_TIMESTAMP_MS
#define APP_LOG_TIMESTAMP_MS 1
#endif
#if (APP_DEBUG_ENABLE)
#define UART_PRINT_DEBUG_ENABLE 1
#define PRINT_BAUD_RATE 1000000
#endif

#ifdef VENDOR_COMMON_TLKAPI_DEBUG_H_
#error "tlkapi_debug.h included before?"
#endif

// we use files from the BLE SDK vendor section "inline"
#include "application/print/putchar.c"
#include "application/print/u_printf.c"
#include "vendor/common/tlkapi_debug.h"
#include "vendor/common/tlkapi_debug.c"
#include "stack/ble/debug/debug.h"

// internal: first log after sleep -> new line
#if (APP_DEBUG_ENABLE)
_attribute_data_retention_ u8 app_dbg_newline = 0;
_attribute_data_retention_ char app_dbg_tstate[2] = {0,0};
#if (APP_LOG_TIMESTAMP_MS)
#endif

void DBGBRK(void)
{
	if (app_dbg_newline!=0) { putchar('\n'); app_dbg_newline = 0; }
	#if (APP_LOG_TIMESTAMP_MS)
	u32 t=clock_time()/16/1000; tlk_printf( "%06u:", t);
    #endif
	u8 tsep=0;
	if (app_dbg_tstate[0])  { putchar(app_dbg_tstate[0]); tsep=1; }
	if (app_dbg_tstate[1])  { putchar(app_dbg_tstate[1]); tsep=1; }
	if (tsep)  putchar(':');
}

void DBGSETTRACESTATE(u8 nr, char state)
{
	if (nr>=sizeof(app_dbg_tstate))  return;
	app_dbg_tstate[nr]=state;
}
#endif

// debug init
void app_debug_init(u8 deepRetn)
{
	// gpio_set_func(DEBUG_SWS_PIN, AS_SWIRE);
    #if (APP_DEBUG_ENABLE)
	gpio_set_func(DEBUG_INFO_TX_PIN, AS_GPIO);
	gpio_write(DEBUG_INFO_TX_PIN, 1);
	gpio_set_output_en(DEBUG_INFO_TX_PIN, 1);
	tlkapi_debug_init();
    blc_debug_enableStackLog(STK_LOG_DISABLE);
	if (deepRetn>0)  app_debug_nextline();
    #endif
}

void app_debug_nextline(void)
{
	#if (APP_DEBUG_ENABLE)
	app_dbg_newline=1;
	#endif
}

// debug helpers
#if (APP_DEBUG_ENABLE)
void DEBUGOUTHEX(u8 u)
{
	u8 h=(u >> 4);
    if (h>=10)  DEBUGOUT(h-10+'A'); else DEBUGOUT(h+'0');
	h=(u & 0x0F);
    if (h>=10)  DEBUGOUT(h-10+'A'); else DEBUGOUT(h+'0');
}

void DEBUGOUTSTR(const char *txt)
{
	while (*txt)  { DEBUGOUT(*txt); txt++; }
}

void DEBUGOUTINT(int val, int digits)
{
	char buf[20]; u8 ofs=sizeof(buf);
	if (digits<1) { digits=1; }
	if (val<0) { DEBUGOUT('-'); val=-val; }
	while (1)
	{
		if (digits<=0 && val==0)   break;
		buf[--ofs]=(char)((val%10)+'0');
		val/=10; digits--;
	}
	while (ofs<sizeof(buf))   DEBUGOUT(buf[ofs++]);
}
#endif








