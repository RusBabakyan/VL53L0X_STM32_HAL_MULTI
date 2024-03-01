# VL53L0X_STM32_HAL_MULTI
VL53L0X ToF Distance Sensor Library

### It is full copy of the VL53L0X ToF Distance Sensor Library - STM32 HAL (https://github.com/Squieler/VL53L0X---STM32-HAL?tab=readme-ov-file)

I have modified this library to allow working with severals sensor at one time. Now any function accept firs parameter of VL53L0X *dev, that contain all neccesary information about sensor:

``` C
typedef struct{
	uint8_t g_i2cAddr;
	uint16_t g_ioTimeout;
	uint8_t g_isTimeout;
	uint16_t g_timeoutStartMs;
	uint8_t g_stopVariable;
	uint32_t g_measTimBudUs;
} VL53L0X;
```
And added checkRangeContinuousMillimeters() function that returns a range reading in millimeters when continuous mode is active without blocking the program.

## Example Usage
You can find fuul programm in the src/main.c
``` C
VL53L0X dev1; //create struct
uint16_t distance;
```
Restart sensor:
``` C
HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
HAL_Delay(10);
HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
HAL_Delay(100);
```
Initialisation:
``` C
  	dev1.g_i2cAddr = 0b01010010;
  	dev1.g_ioTimeout = 0;
  	dev1.g_isTimeout = 0;
	uint8_t new_addr = 0x54;
	setAddress_VL53L0X(&dev1, new_addr);
	initVL53L0X(&dev1, 1, &hi2c1);

	// Configure the sensor for high accuracy and speed in 20 cm.
	setSignalRateLimit(&dev1, 200);
	setVcselPulsePeriod(&dev1, VcselPeriodPreRange, 10);
	setVcselPulsePeriod(&dev1, VcselPeriodFinalRange, 14);
	setMeasurementTimingBudget(&dev1, 150 * 1000UL);
//	startContinuous(&dev1,0); //run for the Continuous mode
```
Read data in loop:
``` C
distance = readRangeSingleMillimeters(&dev1, &distanceStr);
// for the Continuous mode without blocking the program
// error = checkRangeContinuousMillimeters(&dev1, &distanceStr, &distance); 
```
