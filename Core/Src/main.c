/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2021 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under BSD 3-Clause license,
  * the "License"; You may not use this file except in compliance with the
  * License. You may obtain a copy of the License at:
  *                        opensource.org/licenses/BSD-3-Clause
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "cmsis_os.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "math.h"
#include "queue.h"
#include "task.h"
static QueueHandle_t RollCmdQueue = NULL ;
#include "i2c-lcd.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
__STATIC_INLINE void  Configure_TIMPWMOutput(void);
void Roll_Task (void);
void Uart_Rx_Task(void);
void lcd_init (void);
void lcd_send_cmd (char cmd);
void lcd_send_data (char data);
void lcd_send_string (char *str);
void lcd_put_cur (int row, int col);
void lcd_clear (void);

uint8_t readReg(uint8_t addr);
uint8_t rec_BLE;
void writeReg(uint8_t addr, uint8_t val);
void Usart3_Send(char *string);

void ADXL_Task(void);

//void MX_FREERTOS_Init(void);
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void MX_FREERTOS_Init(void);
/* USER CODE BEGIN PFP */
#define SLAVE_ADDRESS_LCD 0x4E
#define SLAVE_ADDRESS_ADXL 0xA7
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_TIM4_Init();
  MX_USART3_UART_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */
  Configure_TIMPWMOutput();

  RollCmdQueue = xQueueCreate(2,sizeof(uint8_t));
  lcd_init();
  lcd_send_string ("HELLO WORLD");

  while(RollCmdQueue==NULL);
  if(xTaskCreate(Roll_Task,"rolltask",128,NULL,tskIDLE_PRIORITY+2, NULL)!= pdPASS){while(1);}
  if(xTaskCreate(Uart_Rx_Task,"uarttask",128,NULL,tskIDLE_PRIORITY+1, NULL)!= pdPASS){while(1);}
  if(xTaskCreate(ADXL_Task,"adxltask",128,NULL,tskIDLE_PRIORITY+1, NULL)!= pdPASS){while(1);}
  //vTaskStartScheduler();


  /* USER CODE END 2 */

  /* Call init function for freertos objects (in freertos.c) */
  MX_FREERTOS_Init();
  /* Start scheduler */
  //osKernelStart();

  /* We should never get here as control is now taken by the scheduler */
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
	  while(!LL_USART_IsActiveFlag_RXNE(USART6));
	  LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_14);
	  rec_BLE=LL_USART_ReceiveData8(USART6);
    /* USER CODE BEGIN 3 */

	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  LL_FLASH_SetLatency(LL_FLASH_LATENCY_0);
  while(LL_FLASH_GetLatency()!= LL_FLASH_LATENCY_0)
  {
  }
  LL_PWR_SetRegulVoltageScaling(LL_PWR_REGU_VOLTAGE_SCALE3);
  LL_PWR_DisableOverDriveMode();
  LL_RCC_HSI_SetCalibTrimming(16);
  LL_RCC_HSI_Enable();

   /* Wait till HSI is ready */
  while(LL_RCC_HSI_IsReady() != 1)
  {

  }
  LL_RCC_SetAHBPrescaler(LL_RCC_SYSCLK_DIV_1);
  LL_RCC_SetAPB1Prescaler(LL_RCC_APB1_DIV_1);
  LL_RCC_SetAPB2Prescaler(LL_RCC_APB2_DIV_1);
  LL_RCC_SetSysClkSource(LL_RCC_SYS_CLKSOURCE_HSI);

   /* Wait till System clock is ready */
  while(LL_RCC_GetSysClkSource() != LL_RCC_SYS_CLKSOURCE_STATUS_HSI)
  {

  }
  LL_SetSystemCoreClock(16000000);

   /* Update the time base */
  if (HAL_InitTick (TICK_INT_PRIORITY) != HAL_OK)
  {
    Error_Handler();
  }
  LL_RCC_SetUSARTClockSource(LL_RCC_USART3_CLKSOURCE_PCLK1);
  LL_RCC_SetUSARTClockSource(LL_RCC_USART6_CLKSOURCE_PCLK2);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C1_CLKSOURCE_PCLK1);
  LL_RCC_SetI2CClockSource(LL_RCC_I2C2_CLKSOURCE_PCLK1);
}

/* USER CODE BEGIN 4 */
__STATIC_INLINE void  Configure_TIMPWMOutput(void)
{
  /*************************/
  /* GPIO AF configuration */
  /*************************/
  /* Enable the peripheral clock of GPIOs */
  LL_AHB1_GRP1_EnableClock(LL_AHB1_GRP1_PERIPH_GPIOD);


  LL_GPIO_InitTypeDef GPIO_InitStruct = {0};
  GPIO_InitStruct.Pin = LL_GPIO_PIN_12;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_13;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_14;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  GPIO_InitStruct.Pin = LL_GPIO_PIN_15;
  GPIO_InitStruct.Mode = LL_GPIO_MODE_ALTERNATE;
  GPIO_InitStruct.Speed = LL_GPIO_SPEED_FREQ_LOW;
  GPIO_InitStruct.OutputType = LL_GPIO_OUTPUT_PUSHPULL;
  GPIO_InitStruct.Pull = LL_GPIO_PULL_NO;
  GPIO_InitStruct.Alternate = LL_GPIO_AF_2;
  LL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /***********************************************/
  /* Configure the NVIC to handle TIM4 interrupt */
  /***********************************************/
  NVIC_SetPriority(TIM4_IRQn, 0);
  NVIC_DisableIRQ(TIM4_IRQn);

  /******************************/
  /* Peripheral clocks enabling */
  /******************************/
  /* Enable the timer peripheral clock */
  LL_APB1_GRP1_EnableClock(LL_APB1_GRP1_PERIPH_TIM4);

  /***************************/
  /* Time base configuration */
  /***************************/
  /* Set counter mode */
  /* Reset value is LL_TIM_COUNTERMODE_UP */
  LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);

  /* Set the pre-scaler value to have TIM1 counter clock equal to 10 kHz */
  LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 10000));

  /* Enable TIM1_ARR register preload. Writing to or reading from the         */
  /* auto-reload register accesses the preload register. The content of the   */
  /* preload register are transferred into the shadow register at each update */
  /* event (UEV).                                                             */
  LL_TIM_EnableARRPreload(TIM4);

  /* Set the auto-reload value to have a counter frequency of 100 Hz */
  /* TIM1CLK = SystemCoreClock / (APB prescaler & multiplier)               */
  uint32_t TimOutClock = SystemCoreClock/2;
  LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100));

  /*********************************/
  /* Output waveform configuration */
  /*********************************/
  /* Set output mode */
  /* Reset value is LL_TIM_OCMODE_FROZEN */
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCMODE_PWM1);
  LL_TIM_OC_SetMode(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCMODE_PWM1);

  /* Set output channel polarity */
  /* Reset value is LL_TIM_OCPOLARITY_HIGH */
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH1, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH2, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH3, LL_TIM_OCPOLARITY_HIGH);
  LL_TIM_OC_SetPolarity(TIM4, LL_TIM_CHANNEL_CH4, LL_TIM_OCPOLARITY_HIGH);

  /* Set compare value to half of the counter period (50% duty cycle ) */
  LL_TIM_OC_SetCompareCH1(TIM4, 0);
  LL_TIM_OC_SetCompareCH2(TIM4, 0);
  LL_TIM_OC_SetCompareCH3(TIM4, 0);
  LL_TIM_OC_SetCompareCH4(TIM4, 0);

  /* Enable TIM1_CCR1 register preload. Read/Write operations access the      */
  /* preload register. TIM4_CCR1~4 preload value is loaded in the active        */
  /* at each update event.                                                    */
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_OC_EnablePreload(TIM4, LL_TIM_CHANNEL_CH4);

  /**************************/
  /* TIM4 interrupts set-up */
  /**************************/
  /* Enable the capture/compare interrupt for channel 1~4*/
  LL_TIM_EnableIT_CC1(TIM4);
  LL_TIM_EnableIT_CC2(TIM4);
  LL_TIM_EnableIT_CC3(TIM4);
  LL_TIM_EnableIT_CC4(TIM4);

  /**********************************/
  /* Start output signal generation */
  /**********************************/
  /* Enable output channel 1~4 */
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH1);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH2);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH3);
  LL_TIM_CC_EnableChannel(TIM4, LL_TIM_CHANNEL_CH4);

  /* Enable counter */
  LL_TIM_EnableCounter(TIM4);

  /* Force update generation */
  LL_TIM_GenerateEvent_UPDATE(TIM4);
}

void Roll_Task (void){
	uint8_t nextCmd =0;
	uint8_t Cmd =0;
	uint8_t speed = 20;
	while(1){


		if(xQueueReceive(RollCmdQueue,&nextCmd, portMAX_DELAY)== pdTRUE){

		if(nextCmd=='x')
			speed -= 10;
		else if(nextCmd=='c')
			speed += 10;
		else
			Cmd = nextCmd;
		}
		if(Cmd=='w'){
			LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD12
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
			LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD13
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_8);
			LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD14
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_9);
			LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD15
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
			}
			//LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_14);
		else if(Cmd=='d')
		{
			LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_0);
			LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD12
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
			LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD13
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_8);//re 1
			LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD14
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_9);//re 2
			LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD15
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
					}

		else if(Cmd=='s')
		{
			LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_7);
			LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD12
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
			LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD13
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_8);//re 1
			LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD14
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_9);//re 2
			LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD15
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
					}
		else if(Cmd=='a')
			{
			LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_7);
			LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD12
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
			LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD13
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_8);//re 1
			LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*(100-speed) / 100));//PD14
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_9);//re 2
			LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*speed / 100));//PD15
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
			}
		else if(Cmd==' ')
		{
			LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD12
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
			LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD13
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_8);
			LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD14
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_9);
			LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD15
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
			}

	}
}

void Uart_Rx_Task(void){
	uint8_t rec;
	//LL_GPIO_TogglePin(GPIOB,LL_GPIO_PIN_7);
	while(1){
		while(!LL_USART_IsActiveFlag_RXNE(USART3));
		rec =LL_USART_ReceiveData8(USART3);
		if(rec=='w'||rec=='a'||rec=='s'||rec=='d'||rec==' '||rec=='x'||rec=='c'){

			xQueueSend(RollCmdQueue,&rec,portMAX_DELAY);
			vTaskDelay(100/portTICK_PERIOD_MS);

		}
		vTaskDelay(1/portTICK_PERIOD_MS);
	}
}
void ADXL_Task(void){
	uint8_t data[6] = {0} ;
		short dataXYZ[3] = {0};
		float data_acc[3] = {0};
		uint8_t datafmt_val;
		float gravity_range = 8.0f;
	  float resol = 1024.0f;	//10-bit
	  float factor = gravity_range / resol;
	  Usart3_Send("----------------------\n\r");
	  uint8_t* name[3];
	  uint8_t ax[] = "Ax";
	  uint8_t ay[] = "Ay";
	  uint8_t az[] = "Az";

	  name[0]=ax;
	  name[1]=ay;
	  name[2]=az;

	  writeReg(0x2D,0x0A);
	  /**
	   *  Clearing bit3 to config sensor as Normal resolution mode.
	   *  Set bit[1:0] = 0x01 to get ±4g range.
	   */
	  datafmt_val = readReg(0x31);
	  datafmt_val = (datafmt_val & ~(1 << 3)) | 0x01 ;
	  writeReg(0x31,datafmt_val);



	  /* Infinite loop */
	  /* USER CODE BEGIN WHILE */
	  while (1)
	  {

		  data[0] = readReg(0x32);
		  data[1] = readReg(0x33);
		  data[2] = readReg(0x34);
		  data[3] = readReg(0x35);
		  data[4] = readReg(0x36);
		  data[5] = readReg(0x37);



	    for(uint8_t k = 0 ; k <3 ; k ++)
	      dataXYZ[k] = ((short)data[2*k+1] << 8) | (short)data[2*k] ;

	      data_acc[0] = dataXYZ[0]*factor;
	      data_acc[1] = dataXYZ[1]*factor;
	      data_acc[2] = dataXYZ[2]*factor;

	      char uartBuf[20]="";
	        for(int i=0;i<3;i++){
	      	  if(data_acc[i]>0){
	      		  int a=(float)data_acc[i];
	      		  int b = ((float)data_acc[i]-a)*1000;
	      		  sprintf ( uartBuf, " =   %d.%d\n\r",a,b);
	      		  Usart3_Send(name[i]);
	      		  Usart3_Send(uartBuf);

	      	  }
	      	  else if(data_acc[i]<0){
	      		  int a=-((float)data_acc[i]);
	      		  int b = -((float)data_acc[i]-a)*1000;
	      		  sprintf ( uartBuf, " =  -%d.%d\n\r",a,b);
	      		  Usart3_Send(name[i]);
	      		  Usart3_Send(uartBuf);
	        }
		}
	        LL_mDelay(1000);
	  }

}
uint8_t readReg(uint8_t addr)
{
	LL_I2C_HandleTransfer(I2C1, SLAVE_ADDRESS_ADXL, LL_I2C_ADDRSLAVE_7BIT, sizeof(addr), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	while(!LL_I2C_IsActiveFlag_TXE(I2C1));
	LL_I2C_TransmitData8(I2C1, addr);
	LL_mDelay(50);
	LL_I2C_HandleTransfer(I2C1, 0xA6, LL_I2C_ADDRSLAVE_7BIT, sizeof(addr), LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_READ);
	while(!LL_I2C_IsActiveFlag_RXNE(I2C1));
	return LL_I2C_ReceiveData8(I2C1);
}
void writeReg(uint8_t addr, uint8_t val)
{
	LL_I2C_HandleTransfer(I2C1, SLAVE_ADDRESS_ADXL, LL_I2C_ADDRSLAVE_7BIT, sizeof(addr)*2, LL_I2C_MODE_AUTOEND, LL_I2C_GENERATE_START_WRITE);
	LL_mDelay(50);
	while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
	LL_I2C_TransmitData8(I2C1, addr);
	LL_mDelay(50);
	while(!LL_I2C_IsActiveFlag_TXIS(I2C1));
	LL_I2C_TransmitData8(I2C1, val);

}

void Usart3_Send(char *string){
    while(*string){
    	while(!LL_USART_IsActiveFlag_TXE(USART3));
        /* ?��?��?�息?�� USART1 */
        //USART_SendData(USART1, (unsigned short int) *string++);
        LL_USART_TransmitData8(USART3, (uint8_t) *string++);

        /* 等�?��?�息?��?��?�畢 */
        //while (USART_GetFlagStatus(USART1, USART_FLAG_TC) == RESET);
    }
}
/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
