# conrroll-a-car-with-omni-wheels-in-FreeRTOS
Running FreeRTOS in STM32CubeIDE, different tasks let the car can be control the direction, and read the sensor data concurrently.   

# I/O Interface
![I/O](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/io.jpg)
![IOpin](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/io%20pin.png)
## LCD
## 3-Axis IMU
## USART
  * Bluetooth
  * PC
## Motor
  The L298N is a dual H-Bridge motor driver which allows speed and direction control of two DC motors at the same time.  
  The module can drive DC motors that have voltages between 5 and 35V, with a peak current up to 2A.  
  L9110 dual-motor driver module is the single four-line two-phase stepper motor
  ### Perameter Settings
  in STM32CubeMX we use clock configuration to set SYSCLK = 20MHz, and use TIM4 parameter setting to set ARR = 100-1 and prescaler = 20000, then we have
  * PWM Frequency = 20MHz / (100 * 20000) = 10Hz
  ``` 
  LL_TIM_SetCounterMode(TIM4, LL_TIM_COUNTERMODE_UP);
  LL_TIM_SetAutoReload(TIM4, __LL_TIM_CALC_ARR(TimOutClock, LL_TIM_GetPrescaler(TIM4), 100)); 
  LL_TIM_SetPrescaler(TIM4, __LL_TIM_CALC_PSC(SystemCoreClock, 20000));
  ```
  ### Omni Motor Action   
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


# Low Layer Library
![LL](https://github.com/wei94424/conrroll-a-car-with-omni-wheels-in-FreeRTOS/blob/master/img/ll.png)
# FreeRTOS
## Task1 - Roll_Task
## Task2 - Uart_Rx_Task
## Task3 - ADXL_Task
