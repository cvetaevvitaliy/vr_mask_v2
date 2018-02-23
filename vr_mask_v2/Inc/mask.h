#ifndef	_MASK_H
#define	_MASK_H
#include "stm32f1xx_hal.h"


void BT_PowerOn(void);
uint16_t hex_to_short( char* buffer);
void Read_Volt(void);
void Read_Temp(void);
uint8_t PowerOn(void);
void Power_off2(void);
void Blink_Led_Blue(void);
void Blink_led_Red(void);
void Power_OFF(void);
uint8_t Timer_int(uint16_t tim);

#endif

