#include "mask.h"

extern uint32_t adc_buffer[2];
extern float adc_value[2];

float volt=0;
float temperature=0;
extern uint8_t powercount;
uint8_t temP;
extern uint8_t b;


#define VOLTAGE_DIVIDER_R1 33000.0
#define VOLTAGE_DIVIDER_R2 100000.0
#define reference_voltage 3.3

void BT_PowerOn(void)
{
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
}




void Read_Volt()
{
		volt = ((adc_value[0]*reference_voltage)/4095)/(VOLTAGE_DIVIDER_R2/(VOLTAGE_DIVIDER_R1+VOLTAGE_DIVIDER_R1));
}

uint8_t  Read_Temp()
{
		
		temperature=(1.34-(adc_value[1]/4096.0*3.33))/0.0043+25;
		temP=temperature;
		return temP;
}


uint8_t PowerOn()
{
	
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		powercount++;
		HAL_Delay(50);
		
		if (powercount>=65)
		{
			
			HAL_Delay(600);
			//HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
			return 0;
		}
		
		
	}
	else {	if(powercount<=0)
						{
							powercount=0;
							
						} else	{powercount--;
							HAL_Delay(250);}
			}
	
	return 1;

}

void Power_off2()
{
		
			PWR->CSR   |= PWR_CSR_EWUP;
			PWR->CR    |= PWR_CR_CWUF;
			PWR->CR = PWR_CR_PDDS | PWR_CR_CWUF;
			HAL_PWR_EnterSTANDBYMode();	
		

}



void Blink_Led_Blue()
{
	
	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
	
	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15))
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_SET);
	}
			
	else 
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
	}

}


void Blink_led_Red()
{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);

	if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_15))
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_SET);
	}
			
	else 
	{
		HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
	}

}





void Power_OFF()
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		powercount++;
		HAL_Delay(50);
		
		if (powercount>=85)
		{
			Power_off2();
			
			HAL_Delay(100);
			
		}
		
		
	}
	else {	if(powercount<=0)
						{
							powercount=0;
							
						} else	{powercount--;
						//	HAL_Delay(250);
							}
			}
	
	


}





