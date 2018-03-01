#include "mask.h"

extern uint32_t adc_buffer[2];
extern float adc_value[2];

float volt=0;
float temperature=0;
extern uint8_t powercount;



#define VOLTAGE_DIVIDER_R1 33000.0
#define VOLTAGE_DIVIDER_R2 100000.0
#define reference_voltage 3.3

void BT_PowerOn(void)
{
HAL_GPIO_WritePin(GPIOA,GPIO_PIN_1,GPIO_PIN_SET);
}


uint16_t hex_to_short(char* buffer)
{
   uint8_t k,i;
   uint16_t  n; 
   
   for(i=0;i<4;i++)
   {
      k=*(buffer+i)-48;
      if (k>9)           //hex digit not a number 
          k-=7;
      k&=0x0f;       //lowercase or wrong hex digit
      n=(n<<4) | k;
   }
   return n;
}


void Read_Volt()
{
		volt = ((adc_value[0]*reference_voltage)/4095)/(VOLTAGE_DIVIDER_R2/(VOLTAGE_DIVIDER_R1+VOLTAGE_DIVIDER_R1));
}

void Read_Temp()
{
		
		temperature=(1.34-(adc_value[1]/4096.0*3.33))/0.0043+25;
}


uint8_t PowerOn()
{
	
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		powercount++;
		HAL_Delay(50);
		
		if (powercount>=125)
		{
			
			HAL_Delay(600);
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
	HAL_GPIO_TogglePin(GPIOC,GPIO_PIN_13);
	HAL_Delay(300);

}


void Blink_led_Red()
{



}

void Power_OFF()
{
	if (HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_0))
	{
		powercount++;
		//HAL_Delay(50);
		
		if (powercount>=10)
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


uint8_t Timer_int(uint16_t tim)
{
	
	


}


