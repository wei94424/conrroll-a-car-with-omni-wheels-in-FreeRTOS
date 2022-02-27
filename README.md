# conrroll-a-car-with-omni-wheels-in-FreeRTOS
Running FreeRTOS in STM32CubeIDE, different tasks let the car can be control the direction, and read the sensor data concurrently.   
# Low Layer Library
![LL](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/ll.png)
we can monitor the registers by listening the flag condition
表格列出所有有用flag
* for I2C reading the location where deposit some useful data
```
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
```
* for I2C writing in objective register, use to initialize IMU
```
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
 ```
* for UART transmitting, 
```
void Usartx_Send(char *string){
    while(*string){
    	while(!LL_USART_IsActiveFlag_TXE(USARTx));
        LL_USART_TransmitData8(USARTx, (uint8_t) *string++);//Write in Transmitter Data Register (Transmit Data value, 8 bits)
    }
} 
```
# I/O Interface
![I/O](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/io.jpg)
![IOpin](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/io%20pin.png)

# using PWM control the motors
   2 L9110 dual-motor driver module is the single four-line two-phase stepper motor, using to control 4 wheels direction and speed.
  ![motor](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/L9110.png)
  ## TIM4 Perameter Settings
  in STM32CubeMX we use clock configuration to set SYSCLK = 20MHz, and use TIM4 parameter setting to set ARR = 100-1 and prescaler = 20000, then we have
  * PWM Frequency = 20MHz / (100 * 20000) = 10Hz
  ``` 
  LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100)); 
  LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 20000));
  ```
  and then we open Channel1~4 as PWM Generation CH1~4, all in PWM mode 1,  
  thus channel1~4 have the corresponding CCR1~4 which will controling the motor speed, due to duty cycle.
  ## Omni Motor Action   
* overall 

  | GPIO | PWM_slowest | PWM_fastest |
  |:------:|:-----:|:-----:|
  | SET(ex:100) | 99 | 0 |
  | RESET(ex:0) | 0 | 99 |  

    * SET->turning in BACKWARD
    * RESET->turning in FORWARD  
* precisely, in every 5m/s speed up/down for the interval increase/decrease  
![omni wheel](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/wheel-rotations.jpg)  
In forward and backward case, 4 wheels will go on same direction and same speed simultaneously.  

    | LCD_VALUE | CCR_VALUE forward | CCR_VALUE backward |
    |:------:|:-----:|:-----:|
    | 10 | 60(0x3c) | 40(0x28) |
    | 20 | 70(0x46) | 30(0x1e) |
    | 30 | 80(0x50) | 20(0x14) |
    | 40 | 90(0x5a) | 10(0x0a) |
    | 50 | 100(0x64) | 0 |
    
  In left and right case, 4 wheels all turn, and the diagonal wheels of both sides are in same direction.
  In diagonal 4 direction case, only 1 side diagonal wheels turn, so it neither going straightly nor side shift. 
    
    | SetCompare | control wheel |
    |:------:|:-----:|
    | CCR1 | right-front |
    | CCR2 | left-rear |
    | CCR3 | left-front |
    | CCR4 | right-rear |
    
  setting LCD speed in 30, and this table is the all actions value of CCR1~4 - GPIO (RESET=0 / SET=100), simply use the four cardinal directions and the four intercardinal directions to represent the action way.
  ![moving derection](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/compass%20rose.png)  
  * turning backward -> 100-speed(20) - SET(100) = -80 (negative value equal to turning backward)
  * turning forward -> speed(80) - RESET(0) = 80 (positive value equal to turning forward)
  * stop -> 0 - RESET(0) = 0 = 100 - SET(100)
 our car are runing in discrete motion, rather than continuous motion, so we put stop function after every motion.    
  ```
 void Stop(void){
	LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD12
	LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
	LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD13
	LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_8);
	LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD14
	LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_9);
	LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*0 / 100));//PD15
	LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
}```

    | cmd | action     | CCR1-GPIO | CCR2-GPIO | CCR3-GPIO | CCR4-GPIO |
    |-----|------------|-----------|-----------|-----------|-----------|
    | w   | N          | 80        | 80        | 80        | 80        |
    | a   | W          | 80        | 80        | -80       | -80       |
    | s   | S          | -80       | -80       | -80       | -80       |
    | d   | E          | -80       | -80       | 80        | 80        |
    | q   | NW         | 80        | 80        | 0         | 0         |
    | z   | SW         | 0         | 0         | -80       | -80       |
    | e   | NE         | 0         | 0         | 80        | 80        |
    | r   | SE         | -80       | -80       | 0         | 0         |
    | t   | TURN_ROUND | -70       | 70        | 70        | -70       |

  
    * ```PWM Duty cycle = CCR / ( ARR + 1 ) = CCR / 100```  
    * initial CCR ```#define SPEED_INIT 80```    
    * value print on LCD = CCR - 50  
# Task1 - Roll_Task (Motor)
 using the command receiving by Uart_Rx_Task, we want to contol the Omni-wheel car in 10 action (include stop)
 * set initial speed, and rising interval, and threshold
 * print speed and control command on LCD
 * Queue
 * due to command, motor turn in corresponding way, like turn around
 ```
   else if(nextCmd=='t')
			{
			LL_TIM_OC_SetCompareCH2(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*70 / 100));//PD13
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_8);//2
			LL_TIM_OC_SetCompareCH3(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*70 / 100));//PD14
			LL_GPIO_ResetOutputPin(GPIOC,LL_GPIO_PIN_9);//1
			LL_TIM_OC_SetCompareCH1(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*30 / 100));//PD12
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_0);//3
			LL_TIM_OC_SetCompareCH4(TIM4, ( (LL_TIM_GetAutoReload(TIM4) + 1 )*30 / 100));//PD15
			LL_GPIO_SetOutputPin(GPIOC,LL_GPIO_PIN_3);//4
			vTaskDelay(1650/portTICK_PERIOD_MS);
			Stop();
			}
   ```
 *we can also check the CCR1~4 in the SFR, it should be the vlaue as we set CCR1 = CCR4 = 0x1e=30 , CCR2 = CCR3 =0x46=70
 ![sfr](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/sfr.png)
 * clear LCD

# Task2 - Uart_Rx_Task
receiving the command from user phone constantly, which use to control the motor action 
  * Bluetooth (USART6)
    * HC-06 
    * Asynchronous mode
    * in Baud Rate 9600, to connect the Bluetooth of user phone
    ```while(!LL_USART_IsActiveFlag_RXNE(USART6));
		     rec =LL_USART_ReceiveData8(USART6);```
# Task3 - ADXL_Task
  * ADXL345 
    * initialize
    * read 3-aixs 16-bits accelemetor data from 6 corresponding 8-bits registers
    * using USART3 to check the 3-axis value on PC
    * print the IMU data on the LCD screen  
  * LCD
  * PC (USART3)
    * Asynchronous mode
    * in Baud Rate 115200, to connect PC for debug about the correction of the IMU data and abilitiy of UART port.
