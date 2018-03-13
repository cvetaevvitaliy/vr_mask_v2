/**
  ******************************************************************************
  * File Name          : main.c
  * Description        : Main program body
  ******************************************************************************
  ** This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether 
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * COPYRIGHT(c) 2018 STMicroelectronics
  *
  * Redistribution and use in source and binary forms, with or without modification,
  * are permitted provided that the following conditions are met:
  *   1. Redistributions of source code must retain the above copyright notice,
  *      this list of conditions and the following disclaimer.
  *   2. Redistributions in binary form must reproduce the above copyright notice,
  *      this list of conditions and the following disclaimer in the documentation
  *      and/or other materials provided with the distribution.
  *   3. Neither the name of STMicroelectronics nor the names of its contributors
  *      may be used to endorse or promote products derived from this software
  *      without specific prior written permission.
  *
  * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
  * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
  * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
  * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
  * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
  * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
  * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
  * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f1xx_hal.h"

/* USER CODE BEGIN Includes */
#include <stdlib.h>
#include <stdio.h>
#include "mask.h"
#include <stdbool.h>
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc1;
DMA_HandleTypeDef hdma_adc1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef hdma_usart2_rx;
DMA_HandleTypeDef hdma_usart2_tx;

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/

#define PowerOdor 315


uint8_t uart_byte[]={0};
uint8_t uart_buff[6];
uint8_t num_byte=0;

//uint8_t sendData[]="MSOK33\n";
char sendData[11]={0};
uint8_t OK[2]="OK";
uint8_t LE[2]="LE";
uint8_t sendData2[]="OK";
uint8_t sendTEMP[]="  \n\r";
char sendBAT[4];
uint8_t power=0;

uint8_t buff_transmit=0;
uint8_t buf_vape[5];
uint8_t buf_healt_left[5];
uint8_t buf_healt_right[5];
uint8_t buf_coller_left[5];
uint8_t buf_coller_right[5];
uint8_t buf_water_left[5];
uint8_t buf_water_right[5];
uint8_t buf_vibro[5];
int8_t fan_smell=0;
volatile  uint16_t tempvibro=0;


bool healt_right=false;
bool healt_left=false;
bool cooler_left=false;
bool cooler_right=false;
bool water_left=false;
bool water_right=false;
bool vibro=false;
bool vape1=false;
bool vape2=false;
bool vape3=false;
bool vape4=false;
bool vape5=false;
bool vape6=false;
bool vape7=false;
bool vape8=false;
bool vape9=false;


float BAT=0;
uint8_t b=0;


uint32_t time_healt_right=0;
uint32_t time_healt_left=0;
uint32_t time_cooler_right=0;
uint32_t time_cooler_left=0;
uint32_t time_water_right=0;
uint32_t time_water_left=0;
uint32_t time_vibro=0;
uint32_t time_vape1=0;
uint32_t time_vape2=0;
uint32_t time_vape3=0;
uint32_t time_vape4=0;
uint32_t time_vape5=0;
uint32_t time_vape6=0;
uint32_t time_vape7=0;
uint32_t time_vape8=0;
uint32_t time_vape9=0;
uint32_t time_sendData=0;
uint32_t time_sendData2=0;
uint32_t time_sendData3=0;


uint32_t time_haelt_rightOff=0;
uint32_t time_haelt_leftOff=0;
uint32_t time_cooler_rightOff=0;
uint32_t time_cooler_leftOff=0;
uint32_t time_water_rightOff=0;
uint32_t time_water_leftOff=0;
uint32_t time_vibro_Off=0;
uint32_t time_vape1_Off=0;
uint32_t time_vape2_Off=0;
uint32_t time_vape3_Off=0;
uint32_t time_vape4_Off=0;
uint32_t time_vape5_Off=0;
uint32_t time_vape6_Off=0;
uint32_t time_vape7_Off=0;
uint32_t time_vape8_Off=0;
uint32_t time_vape9_Off=0;




char temp2[2];
int16_t  temp=0;
uint8_t powercount=0;

uint32_t adc_buffer[2];
float adc_value[2];

uint32_t tick_time;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_DMA_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_USART2_UART_Init(void);
                                    
void HAL_TIM_MspPostInit(TIM_HandleTypeDef *htim);
                                
                                
                                
                                

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */


 





void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef* hadc)
{
		if (hadc->Instance == ADC1)
			for (int i=0; i<2; i++)
		{
					adc_value[i]=adc_buffer[i];
		}

}	
/* USER CODE END 0 */

int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
		PWR->CSR   |= PWR_CSR_EWUP;
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */
			RCC->APB1ENR |= RCC_APB1ENR_PWREN;
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_USART2_UART_Init();

  /* USER CODE BEGIN 2 */


//--------------Вкл---------

	while(PowerOn())
	{
		if(powercount<=0)
		{	Power_off2();}
		//if(powercount>=60)
		//{break;}
	}
	
	
//---------------------------	
	
	
	
	
	HAL_ADC_Start_DMA(&hadc1,adc_buffer,2); // Старт ADC, напряжение и температура 
	
	//-----Стартуем все таймеры на частотах TIM1,TIM2,TIM3 - 45 кГц TIM4 - 110 кГц ----
				HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1); 
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
				HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
        HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);
	//----------------------------------------------------------------------------------	
			//TIM3->CCR2=1000;
		//	TIM3->CCR3=1000;
		//	TIM3->CCR1=1000;
		//	TIM3->CCR4=1000;
		 BT_PowerOn();
		 powercount=0;
		 HAL_Delay(500);
		 HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
		 //	HAL_UART_Transmit_DMA(&huart2, sendData, sizeof(sendData)-1);
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
		
		
		
		
		Power_OFF();
		
		tick_time=HAL_GetTick();
		
				//	sendBAT=(uint8_t*)b;		
		BAT= ((adc_value[0]*3.33)/4096)/(100000.0/(100000.0+33000.0))*10;	
	  b=BAT;	
		//sendBAT=(uint8_t*)b;	
	 // sprintf(sendData,"MS%s:%f\r\n",OK,BAT);				
		//sendBAT[1]=BAT; 
		
		
		
		
		
		
//		
//		if(tick_time-time_sendData>5000)
//			{
//			time_sendData=HAL_GetTick();
//			//HAL_UART_Transmit_DMA(&huart2, sendData, sizeof(sendData)-1);
//		 // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)sendBAT, 4);
//			}
//			
			
			
			
			
			
//			if(tick_time-time_sendData2>5100)
//			{
//			time_sendData2=HAL_GetTick();
//			HAL_UART_Transmit_DMA(&huart2, sendData2, sizeof(sendData)-1);
//		 // HAL_UART_Transmit_DMA(&huart2, (uint8_t*)sendBAT, 4);
//			}
//			if(tick_time-time_sendData3>5200)
//			{
//			time_sendData3=HAL_GetTick();
//		//	HAL_UART_Transmit_DMA(&huart2, sendData, sizeof(sendData)-1);
//		  HAL_UART_Transmit_DMA(&huart2, (uint8_t*)sendBAT, 4);
//			}
			
			
			
			
			
		//----------------Прием данных по UART --------------------------------------------
				HAL_UART_Receive_DMA(&huart2, uart_byte,1);
		
		
				if (uart_byte[0]!=0)
					{
						if(uart_byte[0]==0x0A)num_byte=0;else
						{
								uart_buff[num_byte]=uart_byte[0];
								num_byte++;
								uart_byte[0]=0;
							
								if(num_byte>=5)num_byte=0;
							
						}
					}
		//-----------------------------------------------------------------------------------			
		//if(uart_buff[0]==0x3F){buff_transmit[0]=uart_buff[0];uart_buff[0]=0;}
//		if(uart_buff[0]==0x3F){buff_transmit=0x3F;uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;uart_buff[5]=0;uart_buff[1]=0;}
//		if(buff_transmit==0x3F)
//		{HAL_UART_Transmit_DMA(&huart2, sendData, sizeof(sendData)-1);buff_transmit=0;uart_buff[0]=0;}			
		
			
			
					
		//------------------Разбор данных с буфера ------------------------------------------
		if(uart_buff[0]!=0&&uart_buff[1]!=0&&uart_buff[2]!=0&&uart_buff[3]!=0&&uart_buff[4]!=0)
			{
		
		if(uart_buff[0]==0x3F){buff_transmit=0x3F;uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;uart_buff[5]=0;uart_buff[1]=0;}
		if(buff_transmit==0x3F)
		{
			if(b>34)
				sprintf(sendData,"MSOK%d%d\n\r",b,Read_Temp());
			else
				sprintf(sendData,"MSLE%d\n\r",b);
			HAL_UART_Transmit_DMA(&huart2, (uint8_t*)sendData, sizeof(sendData)-1);
			buff_transmit=0;
			uart_buff[0]=0;
		}		
				
				
		if(uart_buff[0]==0x38){buf_vape[0]=uart_buff[0];buf_vape[1]=uart_buff[1];buf_vape[2]=uart_buff[2];buf_vape[3]=uart_buff[3];buf_vape[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x33){buf_healt_left[0]=uart_buff[0];buf_healt_left[1]=uart_buff[1];buf_healt_left[2]=uart_buff[2];buf_healt_left[3]=uart_buff[3];buf_healt_left[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x34){buf_healt_right[0]=uart_buff[0];buf_healt_right[1]=uart_buff[1];buf_healt_right[2]=uart_buff[2];buf_healt_right[3]=uart_buff[3];buf_healt_right[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x31){buf_coller_left[0]=uart_buff[0];buf_coller_left[1]=uart_buff[1];buf_coller_left[2]=uart_buff[2];buf_coller_left[3]=uart_buff[3];buf_coller_left[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x32){buf_coller_right[0]=uart_buff[0];buf_coller_right[1]=uart_buff[1];buf_coller_right[2]=uart_buff[2];buf_coller_right[3]=uart_buff[3];buf_coller_right[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x36){buf_water_left[0]=uart_buff[0];buf_water_left[1]=uart_buff[1];buf_water_left[2]=uart_buff[2];buf_water_left[3]=uart_buff[3];buf_water_left[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x37){buf_water_right[0]=uart_buff[0];buf_water_right[1]=uart_buff[1];buf_water_right[2]=uart_buff[2];buf_water_right[3]=uart_buff[3];buf_water_right[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		if(uart_buff[0]==0x35){buf_vibro[0]=uart_buff[0];buf_vibro[1]=uart_buff[1];buf_vibro[2]=uart_buff[2];buf_vibro[3]=uart_buff[3];buf_vibro[4]=uart_buff[4];uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}
		else{uart_buff[0]=0;uart_buff[1]=0;uart_buff[2]=0;uart_buff[3]=0;uart_buff[4]=0;}				
			}
			
		
		//-----------------------------------------------------------------------------------

		
			
				if(buf_healt_right[0]==0x34)
						{
							healt_right=true;
							TIM4->CCR4=(654*(buf_healt_right[2]-0x30)/10);
//							TIM3->CCR3=(250*(buf_healt_right[2]-0x30)/10);
							TIM3->CCR3=40;
							buf_healt_right[2]=0;
							time_healt_right=HAL_GetTick();
							buf_healt_right[0]=0;
							time_haelt_rightOff=(buf_healt_right[3]-0x30)*10 + (buf_healt_right[4]-0x30);
							buf_healt_right[3]=0;buf_healt_right[4]=0;buf_healt_right[1]=0;
						}	
					
						if(buf_healt_left[0]==0x33)
						{
							healt_left=true;
//							TIM3->CCR2=(250*(buf_healt_left[2]-0x30)/10);
							TIM3->CCR2=40;
							TIM4->CCR3=(654*(buf_healt_left[2]-0x30)/10);
							buf_healt_left[2]=0;
							time_healt_left=HAL_GetTick();
							buf_healt_left[0]=0;
							time_haelt_leftOff=(buf_healt_left[3]-0x30)*10 + (buf_healt_left[4]-0x30);
							buf_healt_left[3]=0;buf_healt_left[4]=0;buf_healt_left[1]=0;
						}	
						
				
				if(buf_coller_right[0]==0x32)
						{
							cooler_right=true;
							TIM3->CCR3=(1600*(buf_coller_right[2]-0x30)/9);
							buf_coller_right[2]=0;
							time_cooler_right=HAL_GetTick();
							buf_coller_right[0]=0;
							time_cooler_rightOff=(buf_coller_right[3]-0x30)*10 + (buf_coller_right[4]-0x30);
							buf_coller_right[3]=0;buf_coller_right[4]=0;buf_coller_right[1]=0;
						}	
				
						
				if(buf_coller_left[0]==0x31)
						{
							cooler_left=true;
							TIM3->CCR2=(1600*(buf_coller_left[2]-0x30)/9);
							buf_coller_left[2]=0;
							time_cooler_left=HAL_GetTick();
							buf_coller_left[0]=0;
							time_cooler_leftOff=(buf_coller_left[3]-0x30)*10 + (buf_coller_left[4]-0x30);
							buf_coller_left[3]=0;buf_coller_left[4]=0;buf_coller_left[1]=0;
						}	
						
						
				if(buf_water_right[0]==0x37)
						{
							water_right=true;
						//	TIM4->CCR2=327;
						TIM4->CCR2=((322*(buf_water_right[2]-0x30))/10);
							time_water_right=HAL_GetTick();
							buf_water_right[0]=0;
							time_water_rightOff=(buf_water_right[3]-0x30)*10 + (buf_water_right[4]-0x30);
							buf_water_right[3]=0;buf_water_right[4]=0;buf_water_right[1]=0;buf_water_right[2]=0;
						}	
						
						
				if(buf_water_left[0]==0x36)
						{
							water_left=true;
						  //TIM4->CCR1=327;
							TIM4->CCR1=((322*(buf_water_left[2]-0x30))/10);
							time_water_left=HAL_GetTick();
							buf_water_left[0]=0;
							time_water_leftOff=(buf_water_left[3]-0x30)*10 + (buf_water_left[4]-0x30);
							buf_water_left[3]=0;buf_water_left[4]=0;buf_water_left[1]=0;buf_water_left[2]=0;
						}	
						
						
				if(buf_vibro[0]==0x35)
						{
							vibro=true;
							tempvibro=((1000+(buf_vibro[2]-0x30)*90));
							TIM3->CCR4=tempvibro;
						  buf_vibro[2]=0;
							time_vibro=HAL_GetTick();
							buf_vibro[0]=0;
							time_vibro_Off=(buf_vibro[3]-0x30)*10 + (buf_vibro[4]-0x30);
							buf_vibro[3]=0;buf_vibro[4]=0;buf_vibro[1]=0;
						}	
						
			

	if(buf_vape[0]==0x38)
	{
								
				if(buf_vape[1]==0x31)
						{
							if(vape1!=true)
									fan_smell++;
							vape1=true;
							TIM2->CCR2=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
						//	fan_smell++;
							time_vape1=HAL_GetTick();
							buf_vape[0]=0;
							time_vape1_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}	
						
						if(buf_vape[1]==0x32)
						{
							if(vape2!=true)
									fan_smell++;
							vape2=true;
							TIM2->CCR1=(PowerOdor*(buf_vape[2]-0x30)/10);;
						//	fan_smell++;
							time_vape2=HAL_GetTick();
							buf_vape[0]=0;
							time_vape2_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						
						if(buf_vape[1]==0x33)
						{
							if(vape3!=true)
									fan_smell++;
							vape3=true;
							TIM1->CCR4=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
						//	fan_smell++;
							time_vape3=HAL_GetTick();
							buf_vape[0]=0;
							time_vape3_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						if(buf_vape[1]==0x34)
						{
							if(vape4!=true)
									fan_smell++;
							vape4=true;
							TIM1->CCR3=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
							//fan_smell++;
							time_vape4=HAL_GetTick();
							buf_vape[0]=0;
							time_vape4_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						if(buf_vape[1]==0x35)
						{
							if(vape5!=true)
									fan_smell++;
							vape5=true;
							TIM1->CCR2=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
						//	fan_smell++;
							time_vape5=HAL_GetTick();
							buf_vape[0]=0;
							time_vape5_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						if(buf_vape[1]==0x36)
						{
							if(vape6!=true)
									fan_smell++;
							vape6=true;
							TIM1->CCR1=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
							//fan_smell++;
							time_vape6=HAL_GetTick();
							buf_vape[0]=0;
							time_vape6_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						if(buf_vape[1]==0x37)
						{
							if(vape7!=true)
									fan_smell++;
							vape7=true;
							TIM2->CCR4=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
							//fan_smell++;
							time_vape7=HAL_GetTick();
							buf_vape[0]=0;
							time_vape7_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						if(buf_vape[1]==0x38)
						{
							if(vape8!=true)
									fan_smell++;
							vape8=true;
							TIM2->CCR3=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;
							//fan_smell++;
							time_vape8=HAL_GetTick();
							buf_vape[0]=0;
							time_vape8_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
						if(buf_vape[1]==0x39)
						{
							if(vape9!=true)
									fan_smell++;
							vape9=true;
							TIM3->CCR1=(PowerOdor*(buf_vape[2]-0x30)/10);
							buf_vape[2]=0;							
						//	fan_smell++;
							time_vape9=HAL_GetTick();
							buf_vape[0]=0;
							time_vape9_Off=(buf_vape[3]-0x30)*10 + (buf_vape[4]-0x30);
							buf_vape[3]=0;buf_vape[4]=0;buf_vape[1]=0;
						}
						
	}			
				
			
			
			if(healt_right==true&&tick_time-time_healt_right>(time_haelt_rightOff * 100)){TIM4->CCR4=0;healt_right=true; time_healt_right=HAL_GetTick(); if (cooler_right==true)healt_right=true;else {TIM3->CCR3=0;healt_right=false;} }
			if(healt_left==true&&tick_time-time_healt_left>(time_haelt_leftOff * 100)){TIM4->CCR3=0;  healt_left=true; time_healt_left=HAL_GetTick();if (cooler_left==true)healt_left=true;else {TIM3->CCR2=0;healt_left=false;}} 
			if(cooler_left==true&&tick_time-time_cooler_left>(time_cooler_leftOff * 100)){TIM3->CCR2=0;TIM4->CCR3=0;  cooler_left=false; time_cooler_left=HAL_GetTick();}
			if(cooler_right==true&&tick_time-time_cooler_right>(time_cooler_rightOff * 100)){TIM3->CCR3=0;TIM4->CCR4=0;  cooler_right=false; time_cooler_right=HAL_GetTick();}
			if(water_right==true&&tick_time-time_water_right>(time_water_rightOff * 100)){TIM4->CCR2=0;  water_right=false; time_water_right=HAL_GetTick();}
			if(water_left==true&&tick_time-time_water_left>(time_water_leftOff * 100)){TIM4->CCR1=0;  water_left=false; time_water_left=HAL_GetTick();}
			if(vibro==true&&tick_time-time_vibro>(time_vibro_Off * 100)){TIM3->CCR4=0;  vibro=false; time_vibro=HAL_GetTick();tempvibro=0;buf_vibro[2]=0;}
			
			
			
			if(vape1==true&&tick_time-time_vape1>(time_vape1_Off * 100)){TIM2->CCR2=0; vape1=false; time_vape1=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape2==true&&tick_time-time_vape2>(time_vape2_Off * 100)){TIM2->CCR1=0; vape2=false; time_vape2=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape3==true&&tick_time-time_vape3>(time_vape3_Off * 100)){TIM1->CCR4=0; vape3=false; time_vape3=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape4==true&&tick_time-time_vape4>(time_vape4_Off * 100)){TIM1->CCR3=0; vape4=false; time_vape4=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape5==true&&tick_time-time_vape5>(time_vape5_Off * 100)){TIM1->CCR2=0; vape5=false; time_vape5=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape6==true&&tick_time-time_vape6>(time_vape6_Off * 100)){TIM1->CCR1=0; vape6=false; time_vape6=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape7==true&&tick_time-time_vape7>(time_vape7_Off * 100)){TIM2->CCR4=0; vape7=false; time_vape7=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape8==true&&tick_time-time_vape8>(time_vape8_Off * 100)){TIM2->CCR3=0; vape8=false; time_vape8=HAL_GetTick();fan_smell=fan_smell-1;}
			if(vape9==true&&tick_time-time_vape9>(time_vape9_Off * 100)){TIM3->CCR1=0; vape9=false; time_vape9=HAL_GetTick();fan_smell=fan_smell-1;}
		
			if(fan_smell>0) 
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
			}
				else
			{
				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
			}
			
//			if (vape1==true||vape2==true||vape3==true||vape4==true||vape5==true||vape6==true||vape7==true||vape8==true||vape9==true)
//			{
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_SET);
//			}
//				else
//			{
//				HAL_GPIO_WritePin(GPIOB,GPIO_PIN_15,GPIO_PIN_RESET);
//			}
//					
			
			if(b<37)
			{
		//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_14,GPIO_PIN_RESET);
			Blink_led_Red();
			}
			else
			{
		//	HAL_GPIO_WritePin(GPIOC,GPIO_PIN_13,GPIO_PIN_RESET);
			Blink_Led_Blue();
			}
		//	HAL_UART_Transmit_DMA(&huart2, sendData, sizeof(sendData)-1);
		//	HAL_Delay(100);
			
			
			
// 		
//		temp2[0] = uart_buff[0];
//		temp2[1] = uart_buff[1];

//	//	temp = (temp2[0] - 48) * 10 + (temp2[1] - 48);
//		
//		temp = temp2[0];
//		
//		Read_Volt();
//		Read_Temp();
//					Blink_Led_Blue();
//					Power_OFF();
		
  }
  /* USER CODE END 3 */

}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_PeriphCLKInitTypeDef PeriphClkInit;

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV2;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_ADC;
  PeriphClkInit.AdcClockSelection = RCC_ADCPCLK2_DIV6;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC1 init function */
static void MX_ADC1_Init(void)
{

  ADC_ChannelConfTypeDef sConfig;

    /**Common config 
    */
  hadc1.Instance = ADC1;
  hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;
  hadc1.Init.ContinuousConvMode = ENABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 2;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_4;
  sConfig.Rank = 1;
  sConfig.SamplingTime = ADC_SAMPLETIME_239CYCLES_5;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure Regular Channel 
    */
  sConfig.Channel = ADC_CHANNEL_TEMPSENSOR;
  sConfig.Rank = 2;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/* TIM1 init function */
static void MX_TIM1_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;
  TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 0;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 1600;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_DISABLE;
  sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_DISABLE;
  sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
  sBreakDeadTimeConfig.DeadTime = 0;
  sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
  sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
  sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
  if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim1);

}

/* TIM2 init function */
static void MX_TIM2_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 1600;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim2);

}

/* TIM3 init function */
static void MX_TIM3_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 0;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 1600;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim3);

}

/* TIM4 init function */
static void MX_TIM4_Init(void)
{

  TIM_ClockConfigTypeDef sClockSourceConfig;
  TIM_MasterConfigTypeDef sMasterConfig;
  TIM_OC_InitTypeDef sConfigOC;

  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 654;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

  HAL_TIM_MspPostInit(&htim4);

}

/* USART2 init function */
static void MX_USART2_UART_Init(void)
{

  huart2.Instance = USART2;
  huart2.Init.BaudRate = 9600;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

/** 
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void) 
{
  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

  /* DMA interrupt init */
  /* DMA1_Channel1_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel1_IRQn);
  /* DMA1_Channel6_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel6_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel6_IRQn);
  /* DMA1_Channel7_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(DMA1_Channel7_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(DMA1_Channel7_IRQn);

}

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
static void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, LED_STATUS_Pin|LED_STATUS_2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(BLUETOOTH_UP_GPIO_Port, BLUETOOTH_UP_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(Fan_smell_GPIO_Port, Fan_smell_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pins : LED_STATUS_Pin LED_STATUS_2_Pin */
  GPIO_InitStruct.Pin = LED_STATUS_Pin|LED_STATUS_2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : PC15 */
  GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : POWER_KEY_Pin */
  GPIO_InitStruct.Pin = POWER_KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(POWER_KEY_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : BLUETOOTH_UP_Pin */
  GPIO_InitStruct.Pin = BLUETOOTH_UP_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(BLUETOOTH_UP_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : CHRG_STATUS_Pin */
  GPIO_InitStruct.Pin = CHRG_STATUS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(CHRG_STATUS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : Fan_smell_Pin */
  GPIO_InitStruct.Pin = Fan_smell_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(Fan_smell_GPIO_Port, &GPIO_InitStruct);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
