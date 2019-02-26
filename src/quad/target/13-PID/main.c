
#include "system_stm32f4xx.h"
#include "stm32f4xx_rcc.h"
#include "stm32f4xx_gpio.h"
#include "misc.h"
#include "nvic.h"
#include "system.h"
#include "led.h"
#include "sound_beeper.h"
#include "config.h"
#include "configMaster.h"	// including gyro.h, acceleration.h, boardAlignment.h
#include "config_eeprom.h"
#include "exti.h"
#include "button.h"
#include "serial.h"
//#include "msp_serial.h"
//#include "printf.h"
#include "gps.h"
#include "rxSerial1Test.h"
#include "rxSerial3Test.h"
#include "rxSerial6Test.h"
#include <stdio.h>
#include "bus_i2c.h"
#include "bitband_i2c_soft.h"					// self-implemented i2c protocol
#include "bus_spi.h"
#include "initialisation.h"
#include "accgyro_spi_mpu9250.h"
//#include "gyro.h"
//#include "acceleration.h"
//#include "boardAlignment.h"
#include "maths.h"

//#include "mpu6050.h"
//#include "mpu6050_soft_i2c.h"					// for MPU6050 testing purposes
#include "mpu9250_soft_i2c.h"					// for MPU9250 testing purposes
//#include "inv_mpu.h"							// for MPU6050/MPU9250 DMP testing purposes
//#include "inv_mpu_dmp_motion_driver.h"			// for MPU6050/MPU9250 DMP testing purposes

#include "pwm_output.h"			// including timer.h ledTimer.h
#include "rx_pwm.h"
#include "rx.h"
#include "feature.h"

#include "time.h"				// allow to use timeUs_t which is uint32_t
#include "fc_core.h"
#include "fc_tasks.h"           // fcTasksInit()
#include "scheduler.h"          // cfTask_t

#include "mixer.h"

#include "debug.h"

#include "sdcard.h"
#include "asyncfatfs.h"

#include "blackbox.h"
#include "blackbox_io.h"

#include "pid.h"

#include "runtime_config.h"

#include "imu.h"


typedef enum {
	SYSTEM_STATE_INITIALISING			= 0,
	SYSTEM_STATE_CONFIG_LOADED			= (1 << 0),
	SYSTEM_STATE_SENSORS_READY			= (1 << 1),
	SYSTEM_STATE_MOTORS_READY			= (1 << 2),
	SYSTEM_STATE_TRANSPONDER_ENABLED	= (1 << 3),
	SYSTEM_STATE_ALL_READY				= (1 << 7)
}systemState_e;

uint8_t systemState = SYSTEM_STATE_INITIALISING;

void systemInit(void);

#if 1
struct __FILE
{
    int dummy;
};

FILE __stdout;

int fputc(int ch, FILE *f)
{
    /* Send byte to USART */
//	gpsWrite(ch);
//	rxSerial1TestWrite(ch);
	rxSerial3TestWrite(ch);
//	rxSerial6TestWrite(ch);
    
    /* If everything is OK, you have to return character written */
    return ch;
    /* If character is not correct, you can return EOF (-1) to stop writing */
    //return -1;
}
#else
#ifdef __GNUC__
	#define PUTCHAR_PROTOTYPE int __io_putchar(int ch)
#else
	#define PUTCHAR_PROTOTYPE int fputc(int ch, FILE *f)
#endif

PUTCHAR_PROTOTYPE
{
	rxSerial3TestWrite(ch);
	return ch;
}
#endif

//static IO_t mpuSpi9250CsPin = IO_NONE;

//bool sdaFlag = false;

//uint32_t TIM_ARR = 0;
//int16_t ledDutyCycle[LED_NUMBER];

//extern uint8_t TIM_CAPTURE_STATUS;			// status of timer input capture

extern uint8_t motorControlEnable;

//static void comparisonBetweenFastNormalInvSqrt(void)
//{
//	const uint32_t iterations = 90000;
//	const float num = M_PIf;
//	float fastInvSqrtRes, regularInvSqrtRes;
//	
//	uint32_t fastStartTime, fastEndTime, regStartTime, regEndTime;
//	fastStartTime = micros();
//	
//	for (uint32_t i = 0; i < iterations; i++) {
//		fastInvSqrtRes = fastInvSqrt(num + i);
//	}
//	
//	fastEndTime = micros();
//	printf("fastInvSqrt(x) runs in %u us under %u iterations\r\n", (fastEndTime - fastStartTime), iterations);
//	
//	regStartTime = micros();
//	
//	for (uint32_t i = 0; i < iterations; i++) {
//		regularInvSqrtRes = 1/sqrtf(num + i);
//	}
//	
//	regEndTime = micros();
//	printf("1/sqrtf(x) runs in %u us under %u iterations\r\n", (fastEndTime - fastStartTime), iterations);
////	printf("Input: %f, regRes: %f under %u us\r\n", num, regularInvSqrtRes, (regEndTime - regStartTime));

//	if ((regEndTime - regStartTime) > (fastEndTime - fastStartTime)) {
//		printf("fastInvSqrt(x) is %d times faster than 1/sqrtf(x)\r\n", (regEndTime - regStartTime) / (fastEndTime - fastStartTime));
//	} else {
//		printf("1/sqrtf(x) is %d times faster than fastInvSqrt(x)\r\n", (fastEndTime - fastStartTime) / (regEndTime - regStartTime));
//	}	
//}

//static void comparisonBetweenSinApproxAndSinf(void)
//{	
//	const uint32_t iterations = 9000000;
//	const float num = M_PIf * 0.33333333333333f;
//	float sinApproxRes, sinfRes;
//	
//	uint32_t sinApproxStartTime, sinApproxEndTime, sinfStartTime, sinfEndTime;
//	
//	sinApproxStartTime = micros();
//	
//	for (uint32_t i = 0; i < iterations; i++) {
//		sinApproxRes = sinApprox(num);
//	}
//	
//	sinApproxEndTime = micros();
//	printf("sinApprox(x) runs in %u us under %u iterations, res: %f\r\n", (sinApproxEndTime - sinApproxStartTime), iterations, sinApproxRes);
//	
//	sinfStartTime = micros();
//	
//	for (uint32_t i = 0; i < iterations; i++) {
//		sinfRes = sinf(num);
//	}
//	
//	sinfEndTime = micros();
//	printf("sinf(x) runs in %u us under %u iterations, res: %f\r\n", (sinfEndTime - sinfStartTime), iterations, sinfRes);

//	if ((sinfEndTime - sinfStartTime) > (sinApproxEndTime - sinApproxStartTime)) {
//		printf("sinApprox(x) is %d times faster than sinf(x)\r\n", (sinfEndTime - sinfStartTime) / (sinApproxEndTime - sinApproxStartTime));
//	} else {
//		printf("sinf(x) is %d times faster than sinApprox(x)\r\n", (sinApproxEndTime - sinApproxStartTime) / (sinfEndTime - sinfStartTime));
//	}	
//}


//void DCBrushedMotor_PWM_Init(uint16_t arr, uint16_t psc)
//{
//	RCC->APB2ENR|=1<<11;       //使能TIM1时钟
////	RCC->APB2ENR|=1<<2;        //PORTA时钟使能
////	GPIOA->CRH&=0XFFFF0FF0;    //PORTA8 11复用输出
////	GPIOA->CRH|=0X0000B00B;    //PORTA8 11复用输出
//	TIM1->ARR=arr;             //设定计数器自动重装值
//	TIM1->PSC=psc;             //预分频器不分频
//	TIM1->CCMR2|=6<<12;        //CH4 PWM1模式
//	TIM1->CCMR1|=6<<4;         //CH1 PWM1模式
//	TIM1->CCMR2|=1<<11;        //CH4预装载使能
//	TIM1->CCMR1|=1<<3;         //CH1预装载使能
//	TIM1->CCER|=1<<12;         //CH4输出使能
//	TIM1->CCER|=1<<0;          //CH1输出使能
//	TIM1->BDTR |= 1<<15;       //TIM1必须要这句话才能输出PWM
//	TIM1->CR1=0x8000;          //ARPE使能
//	TIM1->CR1|=0x01;           //使能定时器1
//}

//IO_t AIN1, AIN2, BIN1, BIN2, PWMA, PWMB;

void main_process(void)
{
    scheduler();
}

int main(void)
{
//	printfSupportInit();
	
	systemInit();
	
	/* Initialise IO (needed for all IO operations) */
	IOGlobalInit();
	
	/* Initialise EEPROM */
	initEEPROM();
	
	/* Check if EEPROM contains valid data */
	checkEEPROMContainsValidData();

	/* Initialise serial port usage list */
	serialInit(SerialConfig());

	/* Initialise debugging serial port */
//	rxSerial1TestInit();
	rxSerial3TestInit();
//	rxSerial6TestInit();
	
	/* Write masterConfig info into FLASH EEPROM
	 *
	 * TODO: (ISSUE) After calling writeEEPROM() function, the program will not be running when STM32F4 board is powered on
	 */
//	writeEEPROM();	// TODO: writeEEPROM() should be included inside the checkEEPROMContainsValidData() function
					//       separated here just for using printf (serialInit and rxSerial3TestInit initialised before writeEEPROM() and readEEPROM())
	
	/* Read masterConfig info from FLASH EEPROM */
	readEEPROM();
	
	systemState |= SYSTEM_STATE_CONFIG_LOADED;
	
	debugMode = masterConfig.debug_mode;
	
	/* IMPORTANT: 
	 * 		DO NOT FORGET TO CALL latchActiveFeatures() function to perform the following action
	 *			activeFeaturesLatch = masterConfig.enabledFeatures
	 * Latch active features to be used for feature() in the remainder of init()
	 */
	latchActiveFeatures();
//	printf("masterConfig.enabledFeatures: 0x%x, %s, %d\r\n", masterConfig.enabledFeatures, __FUNCTION__, __LINE__);		// 0x2000 (1 << 13) FEATURE_RX_PARALLEL_PWM
	
	/* Initialise leds */
	LedInit(LedStatusConfig());
	
	/* Initialise external interrupt */
	EXTIInit();
	
	/* allow configuration to settle */
	delay(100);


	/* Timer must be initialised before any channel is allocated */
	timerInit();					// reinitialise the LED IO configuration to timer AF_PP if USE_LEDTIMER has been set.
									// INFO: To use NORMAL LEDs, turn off the USE_LEDTIMER micro in target.h

	/* DC brushed motor init */
	
//#define DC_BRUSHED_MOTOR1_AIN1	PB13
//#define DC_BRUSHED_MOTOR1_AIN2	PB12
//#define DC_BRUSHED_MOTOR2_BIN1	PB14
//#define DC_BRUSHED_MOTOR2_BIN2	PB15
	
//	AIN1 = IOGetByTag(IO_TAG(DC_BRUSHED_MOTOR1_AIN1));
//	AIN2 = IOGetByTag(IO_TAG(DC_BRUSHED_MOTOR1_AIN2));
//	BIN1 = IOGetByTag(IO_TAG(DC_BRUSHED_MOTOR2_BIN1));
//	BIN2 = IOGetByTag(IO_TAG(DC_BRUSHED_MOTOR2_BIN2));
//	IOInit(AIN1, OWNER_MOTOR, 0);		// no need
//	IOInit(AIN2, OWNER_MOTOR, 1);		// no need
//	IOInit(BIN1, OWNER_MOTOR, 2);		// no need
//	IOInit(BIN2, OWNER_MOTOR, 3);		// no need
//	IOConfigGPIO(AIN1, IOCFG_OUT_PP);
//	IOConfigGPIO(AIN2, IOCFG_OUT_PP);
//	IOConfigGPIO(BIN1, IOCFG_OUT_PP);
//	IOConfigGPIO(BIN2, IOCFG_OUT_PP);
	
//	printf("size: %u\r\n", sizeof(timerHardware));
//	printf("size: %u\r\n", sizeof(timerHardware) / sizeof(timerHardware[0]));
//	PWMA = IOGetByTag(timerHardware[2].tag);
//	PWMB = IOGetByTag(timerHardware[3].tag);
//	IOInit(PWMA, OWNER_MOTOR, 4);
//	IOInit(PWMB, OWNER_MOTOR, 5);
//	IOConfigGPIOAF(PWMA, IOCFG_AF_PP, timerHardware[2].alternateFunction);
//	IOConfigGPIOAF(PWMB, IOCFG_AF_PP, timerHardware[3].alternateFunction);
	
	/* Timer ARR = 7200 */
	dcBrushedMotorInit(DCBrushedMotorConfig());

//	DCBrushedMotor_PWM_Init(7199, 0);

//	IO_t encoder1Tag = IOGetByTag(timerHardware[0].tag);
//	IO_t encoder2Tag = IOGetByTag(timerHardware[1].tag);
//	IOInit(encoder1Tag, OWNER_ENCODERINPUT, 0);
//	IOInit(encoder2Tag, OWNER_ENCODERINPUT, 1);
//	IOConfigGPIOAF(encoder1Tag, IOCFG_AF_PP_PD, timerHardware[0].alternateFunction);
//	IOConfigGPIOAF(encoder2Tag, IOCFG_AF_PP_PD, timerHardware[1].alternateFunction);

	/* Initialise Timer Encoder Interface Mode for Incremental Encoders attached on DC Brushed Motors */
	pwmEncoderInit(PwmEncoderConfig());
//	Encoder_Init_TIM2();
	
#ifdef USE_SPI			// USE_SPI is defined in target.h
	#ifdef USE_SPI_DEVICE_1
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_1);		// SPIDEV_1 = 0
	#endif
	#ifdef USE_SPI_DEVICE_2
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
		spiInit(SPIDEV_2);
	#endif
#endif

#ifdef BEEPER
	beeperInit(BeeperConfig());
#endif

#if defined(BEEPER)
	/* Board power on beeper */
	for (int i = 0; i < 10; i++) {
		delay(25);
		BEEP_ON;
		delay(25);
		BEEP_OFF;
	}
#endif

#ifdef USE_I2C			// USE_I2C is defined in target.h
	/* Initialise I2C device */
	i2cInit(I2C_DEVICE);
#endif

	/* board alignment */
////	initBoardAlignment(BoardAlignment());
	
	/* Testing alignBoard() */
//	int32_t vec[3] = { 1, 2, 3 };
//	alignBoard(vec);
	
	/* Compare FastInvSqrt(x) and 1/sqrtf(x) */
//	comparisonBetweenFastNormalInvSqrt();
//	comparisonBetweenSinApproxAndSinf();

#if 0
#if defined(USE_IMU)			// USE_IMU is defined in target.h
	if (!sensorsAutodetect(GyroConfig(), AccelerometerConfig())) {
		//failureMode();
//		printf("Failed to initialise IMU!, %s, %d\r\n", __FUNCTION__, __LINE__);
		while (1) {
			/* BLUE LED */
			LED5_ON;
			delay(100);
			LED5_OFF;
			delay(100);
		}
	}
#endif
#endif
	
	systemState |= SYSTEM_STATE_SENSORS_READY;

	/* PID configurations
	 * As gyro.targetLooptime is returned from gyroSetSampleRate() function of gyro_sync.c, so we can safely call pidSetTargetLooptime() function now
	 */
//	printf("gyro.targetLooptime: %u\r\n", gyro.targetLooptime);								// 1000 for F450
//	printf("PidConfig()->pid_process_denom: %u\r\n", PidConfig()->pid_process_denom);		// 2 for F450
	
	/* Initialise PID looptime
	 * 
	 * PID looptime = 125 * 4 = 500 us (2 KHz) for F210 racing quad
	 * PID looptime = 1000 * 2 = 2000 us (500 Hz) for F450 racing quad
	 */
	pidSetTargetLooptime(gyro.targetLooptime * PidConfig()->pid_process_denom);
	
	/* Initialise PID filters */
	pidInitFilters(&currentProfile->pidProfile);
	
	/* Initialise PID configurations */
	pidInitConfig(&currentProfile->pidProfile);
	
	imuInit();
	
	/* MCU Support Package(MSP) initialisation */
//	mspFcInit();
	
	/* Testing standard deviation functions */
//	stdev_t var;
//	float dataset[] = {727.7, 1086.5, 1091.0, 1361.3, 1490.5, 1956.1};
//	devClear(&var);
//	for (int i = 0; i < sizeof(dataset)/sizeof(dataset[0]); i++) {
//		devPush(&var, dataset[i]);
//	}
//	float dev = devStandardDeviation(&var);
//	printf("dev: %f, %s, %d\r\n", dev, __FUNCTION__, __LINE__);					// 420.962463
//	printf("devint: %ld, %s, %d\r\n", lrintf(dev), __FUNCTION__, __LINE__);		// 421

#if 0
	/* Initialise RX receiver */
	rxInit(RxConfig(), ModeActivationProfile()->modeActivationConditions);
#endif

#if 0

#ifdef USE_SDCARD
	if (feature(FEATURE_SDCARD) && BlackboxConfig()->device == BLACKBOX_SDCARD) {
//		printf("USE_SDCARD, %s, %d\r\n", __FUNCTION__, __LINE__);
		sdcardInsertionDetectInit();
		sdcard_init(SdcardConfig()->useDma);		// SdcardConfig()->useDma = false for now, use DMA later
		afatfs_init();
//		if (!sdcard_isInserted()) {
//			printf("SDCARD is not present!\r\n");
//		}else {
//			printf("SDCARD is present!\r\n");
//		}
	}
#endif
	
#ifdef BLACKBOX
	initBlackbox();
#endif
	
#endif

#if defined(USE_IMU)
	/* set gyro calibration cycles */
	gyroSetCalibrationCycles();
#endif

	DISABLE_ARMING_FLAG(PREVENT_ARMING);
	
//	const timeUs_t currentTimeUsAfter = micros();
//	printf("currentTimeUsAfter processRx: %u, %s, %d\r\n", currentTimeUsAfter, __FUNCTION__, __LINE__);

	/* Latch active features again as some of them are modified by init() */
	latchActiveFeatures();
	
	/* Set the motorControlEnable flag to be TRUE to control the motors */
	motorControlEnable = true;

//	uint16_t ledpwmval = 1500;
//	uint8_t dir = 1;
	
//	printf("sizeof(master_t): %u\r\n", sizeof(master_t));			// sizeof(master_t): 868 so far

//    extern cfTask_t *taskQueueArray[TASK_COUNT + 1];

    /* Initialise all the RTOS tasks */
    fcTasksInit();
	
	systemState |= SYSTEM_STATE_ALL_READY;
    
//    printf("taskQueueArray[0]->taskName = %s\r\n", taskQueueArray[0]->taskName);                        // taskName = "SYSTEM"
//    printf("taskQueueArray[0]->desiredPeriod = %u\r\n", taskQueueArray[0]->desiredPeriod);              // desiredPeriod = 100000
//    printf("taskQueueArray[0]->staticPriority = %u\r\n\r\n", taskQueueArray[0]->staticPriority);        // staticPriority = 4

//    printf("taskQueueArray[1]->taskName = %s\r\n", taskQueueArray[1]->taskName);                        // taskName = "LED"
//    printf("taskQueueArray[1]->desiredPeriod = %u\r\n", taskQueueArray[1]->desiredPeriod);              // desiredPeriod = 500000
//    printf("taskQueueArray[1]->staticPriority = %u\r\n", taskQueueArray[1]->staticPriority);            // staticPriority = 4

//    printf("taskName\t taskPeriod\r\n");

//	printf("IO_GPIO(AIN1): 0x%x\r\n", (uint32_t)IO_GPIO(DCBrushedMotorConfig()->AIN1));
//	printf("IO_GPIO(AIN2): 0x%x\r\n", (uint32_t)IO_GPIO(DCBrushedMotorConfig()->AIN2));
//	printf("IO_Pin(AIN1): %u\r\n", IO_Pin(DCBrushedMotorConfig()->AIN1));
//	printf("IO_Pin(AIN2): %u\r\n", IO_Pin(DCBrushedMotorConfig()->AIN2));

//	IO_t l_AIN1 = IOGetByTag(DCBrushedMotorConfig()->AIN1);		// PE3
//	IO_t l_AIN2 = IOGetByTag(DCBrushedMotorConfig()->AIN2);		// PE4

	/* Main loop */
	while (1) {

//		IOWrite(DCBrushedMotorConfig()->AIN1, true);			// clear AIN1 to LOW		PE3  BLUE
//		IOWrite(DCBrushedMotorConfig()->AIN2, false);			// set AIN2 to HIGH			PE4  YELLOW

//		IOLo(l_AIN1);
//		IOHi(l_AIN2);
//		IOLo(l_AIN2);
//		IOHi(l_AIN1);
		
//		delay(500);
        
        main_process();

//		delay(10);

//		if (dir) {
//			ledpwmval++;
//		}else {
//			ledpwmval--;
//		}
//		
//		if (ledpwmval > 2000) {
//			dir = 0;
//		}
//		
//		if (ledpwmval == 0) {
//			dir = 1;
//		}

//		for (int i = 0; i < getMotorCount(); i++) {
//			pwmWriteMotor(i, ledpwmval);
//		}
//		
////		for (int i = 0; i < LED_NUMBER; i++) {
////			pwmWriteLed(i, ledpwmval);
////		}
				
#if 0
		const timeUs_t currentTimeUs = micros();
//		printf("currentTimeUs before processRx: %u, %s, %d\r\n", currentTimeUs, __FUNCTION__, __LINE__);
		
//		rxSignalReceived = rxSignalReceivedNotDataDriven;		// rxSignalReceivedNotDataDriven initial false
//		rxIsInFailsafeMode = rxIsInFailsafeModeNotDataDriven;	// rxIsInFailsafeModeNotDataDriven initial true
		
		/* TODO: rxUpdateCheck() function will be running in a separate RTOS task */
		rxUpdateCheck(currentTimeUs, 0);			// update the rxSignalReceivedNotDataDriven, rxIsInFailsafeModeNotDataDriven, rxSignalReceived and rxIsInFailsafeMode
	
		taskUpdateRxMain(currentTimeUs);		// calling processRx(), updateRcCommands()
//		processRx(currentTimeUs);		// processRx function is called inside the taskUpdateRxMain function
		
		taskMainPidLoop(currentTimeUs);
		
//		delay(200);			// taskUpdateRxMain() update period = 200 ms (testing rx ONLY)
		delay(20);				// taskUpdateRxMain() update period = 20 ms (20000 us = 50 Hz), for MOTORS testing
#endif
		
#ifdef 0
//		BEEP_ON;		// turn beeper on
//		delay(200);
//		BEEP_OFF;		// turn beeper off
//		delay(200);

	for (int i = 0; i < 10; i++) {
//		LED3_TOGGLE;
//		LED4_TOGGLE;
		delay(25);
		BEEP_ON;
		delay(25);
		BEEP_OFF;
	}
	delay(2000);
#endif
		
//		gyroUpdate();													// read gyro data
//		delay(100);
#if 0
		gyroUpdate();													// read gyro data
//		accUpdate(&AccelerometerConfig()->accelerometerTrims);			// read acc data
		if (gyro.dev.mpuDetectionResult.sensor == MPU_60x0 && gyro.dev.calibrationFlag) {
//			printf("MPU6050 data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU6050 data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_I2C) {
		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_I2C && gyro.dev.calibrationFlag) {
			printf("MPU9250 (I2C) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (I2C) data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (I2C) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
		}else if (gyro.dev.mpuDetectionResult.sensor == MPU_9250_SPI && gyro.dev.calibrationFlag) {
//			printf("MPU9250 (SPI) data - gyroADCRaw[X]: %d, gyroADCRaw[Y]: %d, gyroADCRaw[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.dev.gyroADCRaw[X], gyro.dev.gyroADCRaw[Y], gyro.dev.gyroADCRaw[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
//			printf("MPU9250 (SPI) data - gyroADCCali[X]: %d, gyroADCCali[Y]: %d, gyroADCCali[Z]: %d, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyroADC[X], gyroADC[Y], gyroADC[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
			printf("MPU9250 (SPI) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z]);
//			printf("MPU9250 (SPI) data - gyroADCfiltered[X]: %.4f, gyroADCfiltered[Y]: %.4f, gyroADCfiltered[Z]: %.4f, accADC[X]: %d, accADC[Y]: %d, accADC[Z]: %d\r\n", gyro.gyroADCf[X], gyro.gyroADCf[Y], gyro.gyroADCf[Z], acc.dev.ADCRaw[X], acc.dev.ADCRaw[Y], acc.dev.ADCRaw[Z]);
		}
//		printf("gyro.dev.gyroADCRaw[X]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[X], __FUNCTION__, __LINE__);
//		printf("gyro.dev.gyroADCRaw[Y]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[Y], __FUNCTION__, __LINE__);
//		printf("gyro.dev.gyroADCRaw[Z]: %d, %s, %d\r\n", gyro.dev.gyroADCRaw[Z], __FUNCTION__, __LINE__);
		delay(50);
#endif
		
#if 0
		if (mpu_dmp_get_data(&pitch, &roll, &yaw) == 0) {
			/* Get raw data of MPU6050 (gyroscope, accelerometer, temperature) */
			MPU6050_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
			MPU6050_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
			temp = MPU6050_Get_Temperature_Data();
			/* Send data to ANO flight controller software */
			mpu6050_send_data(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z);
			usart3_report_imu(acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, (int)(roll*100), (int)(pitch*100), (int)(yaw*10));
			/* Display on serial terminal */
//			printf("MPU6050 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);
//			printf("Euler angles - roll: %.4f, pitch: %.4f, yaw: %.4f\r\n\r\n", roll, pitch, yaw);
//			delay(100);
		}else {
//			printf("mpu_dmp_get_data failed, %s, %d\r\n", __FUNCTION__, __LINE__);
//			delay(100);
		}
#endif

#if 0
		MPU6050_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
		MPU6050_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
		temp = MPU6050_Get_Temperature_Data();
//		printf("MPU6050 gyro data: gyro_x: %d, gyro_y: %d, gyro_z: %d\r\n", gyro_x, gyro_y, gyro_z);
//		printf("MPU6050 temp data: %.2f\r\n", temp);
		printf("MPU6050 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);		
		delay(100);
#endif

#if 0
		MPU9250_Get_Gyroscope_Data(&gyro_x, &gyro_y, &gyro_z);
		MPU9250_Get_Accelerometer_Data(&acc_x, &acc_y, &acc_z);
		temp = MPU9250_Get_Temperature_Data();
//		printf("MPU6050 gyro data: gyro_x: %d, gyro_y: %d, gyro_z: %d\r\n", gyro_x, gyro_y, gyro_z);
//		printf("MPU6050 temp data: %.2f\r\n", temp);
		printf("MPU9250 data - gyro_x: %d, gyro_y: %d, gyro_z: %d, acc_x: %d, acc_y: %d, acc_z: %d, temp: %.2f\r\n", gyro_x, gyro_y, gyro_z, acc_x, acc_y, acc_z, temp);		
		delay(100);
#endif

#if 0
//#ifdef USE_PWM			// USE_PWM is defined in target.h
		/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode
		 * Make sure to use the following uint32_t type for captures
		 * 	-- uint32_t captures[PWM_PORTS_OR_PPM_CAPTURE_COUNT];
		 *  -- static void pwmEdgeCallback(timerCCHandlerRec_t *cbRec, timCCR_t capture)
		 *  -- static void pwmOverflowCallback(timerOvrHandlerRec_t *cbRec, timCCR_t capture)
		 *  -- 	timCCR_t rise;				// timCCR_t is uint32_t
		 *  -- 	timCCR_t fall;				// timCCR_t is uint32_t
		 *  -- 	timCCR_t capture;				// timCCR_t is uint32_t
		 *  -- 	//	captureCompare_t rise;		// captureCompare_t is uint16_t
		 *  -- 	//	captureCompare_t fall;
		 *  -- 	//	captureCompare_t capture;
		 */
		delay(10);
//		printf("TIM_CAPTURE_STATUS: 0x%x, %s, %d\r\n", TIM_CAPTURE_STATUS, __FUNCTION__, __LINE__);
		if (TIM_CAPTURE_STATUS & 0x80) {			// check if the time period between the rising and falling edge of the signal is captured.
			/* print the time period */
			printf("The period of signal high: %u us\r\n", pwmRead(TIM_Channel_1));		// TIM_Channel_1 = 0x0
			TIM_CAPTURE_STATUS = 0x0;				// clear the status to zero for the next timer input capture
		}
		
		/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
//		delay(10);
//		if (TIM5_CH1_CAPTURE_STATUS & 0x80) {		// 0x80: 1 << 7 (capture successful)
//			printf("TIM5_CH1_CAPTURE_VAL: %u\r\n", TIM5_CH1_CAPTURE_VAL);
////			temp = TIM5_CH1_CAPTURE_STATUS & 0x3F;
////			temp *= 0xFFFFFFFF;						// sum of overflow value
//			temp += TIM5_CH1_CAPTURE_VAL;			// retrieve total time period of high voltage level
//			printf("HIGH: %lld us\r\n", temp);
//			TIM5_CH1_CAPTURE_STATUS = 0;			// clear capture status to enable the next input capture
//		}
	/* Testing USER button (PA0) high voltage level time period using TIM5 Input Capture mode */
#endif
		
		/* +----------------------------------------------------------------------------------------+ */
		/* +------------------------------- bus_i2c_soft API testing -------------------------------+ */
		/* +----------------------------------------------------------------------------------------+ */
//		flag = I2C_Start();			// test ok
//		printf("I2C_Start flag: %d, %s, %d\r\n", flag, __FUNCTION__, __LINE__);
//		I2C_Stop();					// test ok
//		I2C_Ack();					// test ok
//		I2C_NoAck();				// test ok
//		flag = I2C_WaitAck();		// test ok
//		printf("I2C_WaitAck flag: %d, %s, %d\r\n", flag, __FUNCTION__, __LINE__);
//		I2C_SendByte(0x59);			// test ok, sends 'Y'
		/* +----------------------------------------------------------------------------------------+ */
		/* +------------------------------- bus_i2c_soft API testing -------------------------------+ */
		/* +----------------------------------------------------------------------------------------+ */
		
//		IIC_Start();
//		delay(1000);

//		IIC_Stop();
//		delay(1000);

//		IIC_Ack();
//		delay(1000);
//		IIC_NAck();
//		delay(1000);
		
//		IIC_SCL = 0;
//		delay(500);
//		IIC_SCL = 1;
//		delay(500);
		
//		IIC_SDA = 0;
//		delay(800);
//		IIC_SDA = 1;
//		delay(800);

//		printf("STM32F407 by QUADYANG using USART3 Port\r\n");
//		delay(1000);
		
//		IOHi(g_sda);
//		IOHi(g_scl);
////		I2C_delay();
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));
		
//		IOHi(g_scl);
//		IOHi(g_sda);
//		delay(500);
//		printf("scl: %d\r\n", IORead(g_scl));
//		printf("sda: %d\r\n", IORead(g_sda));
//		IOLo(g_scl);
//		IOLo(g_sda);
//		delay(500);
//		printf("scl: %d\r\n", IORead(g_scl));
//		printf("sda: %d\r\n", IORead(g_sda));

//		IOHi(g_sda);
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));

//		IOLo(g_sda);
//		delay(500);
//		printf("sda: %d\r\n", IORead(g_sda));		
		
//		IOHi(g_sda);
//		IOHi(g_scl);
////		I2C_delay();
//		delay(1000);
//		sdaFlag = IORead(g_sda);
//		printf("sdaFlag: %d\r\n", sdaFlag);
		
//		I2C_Start();
//		delayMicroseconds(3);
//		I2C_Stop();
//		delayMicroseconds(3);

//		printf("STM32F407 by QUADYANG using USART1 Port\r\n");
//		printf("%s, %d\r\n", __FUNCTION__, __LINE__);
//		f_cnt += 0.01;
//		printf("i_cnt: %d, f_cnt: %.4f\r\n", i_cnt++, f_cnt);
//		delay(1000);

//		IOWrite(yellowLedPin, false);		// high
//		delay(1000);
//		IOWrite(yellowLedPin, true);		// low
//		delay(1000);

//		mpu9250WriteRegister(0x1B, 0x10);
//		delay(1000);
//		uint8_t in;
//		mpu9250ReadRegister(0x1B, 1, &in);
//		printf("in: 0x%x\r\n", in);
		
//		IOLo(mpuSpi9250CsPin);
//		delayMicroseconds(1);
////		delay(1000);
//		spiTransferByte(MPU9250_SPI_INSTANCE, 0x1B);
//		IOHi(mpuSpi9250CsPin);
//		delayMicroseconds(1);

//		IOLo(mpuSpi9250CsPin);
//		delay(800);
//		IOHi(mpuSpi9250CsPin);
//		delay(800);
		
//		printf("QUADYANG\n");
//		delay(1000);
		//counter++;
				
//		userBtnPollOps();

//		gpioPA1Ops();
//		gpioPB8Ops();
		
//		LED3_ON;
//		delay(1000);
//		LED3_OFF;
//		delay(1000);
		
//		LED4_ON;
//		delay(700);
//		LED4_OFF;
//		delay(700);

//		LED5_ON;
//		delay(400);
//		LED5_OFF;
//		delay(400);

//		LED6_ON;
//		delay(100);
//		LED6_OFF;
//		delay(100);
		
//		msElapse = millis();
//		currentTimeUs = micros();
	}
//	return 0;
}

void EnableGPIOClk(void)
{
	/* AHB1 clocks enable */
	RCC_AHB1PeriphClockCmd(
		RCC_AHB1Periph_GPIOA 	|
		RCC_AHB1Periph_GPIOB 	|
		RCC_AHB1Periph_GPIOC 	|
		RCC_AHB1Periph_GPIOD 	|
		RCC_AHB1Periph_GPIOE 	|
		RCC_AHB1Periph_GPIOH 	|
		RCC_AHB1Periph_CRC	 	|
		RCC_AHB1Periph_FLITF 	|
		RCC_AHB1Periph_SRAM1	|
		RCC_AHB1Periph_SRAM2	|
		RCC_AHB1Periph_BKPSRAM	|
		RCC_AHB1Periph_DMA1		|
		RCC_AHB1Periph_DMA2		|
		0, ENABLE
	);
	
	/* AHB2 clocks enable */
	RCC_AHB2PeriphClockCmd(0, ENABLE);
	
	/* APB1 clocks enable */
	RCC_APB1PeriphClockCmd(
		RCC_APB1Periph_PWR		|
//		RCC_APB1Periph_I2C3		|	
//		RCC_APB1Periph_I2C2		|	
//		RCC_APB1Periph_I2C1		|	
		RCC_APB1Periph_USART2	|	
		RCC_APB1Periph_SPI3		|	
		RCC_APB1Periph_SPI2		|	
		RCC_APB1Periph_WWDG		|	
		RCC_APB1Periph_TIM5		|	
		RCC_APB1Periph_TIM4		|	
		RCC_APB1Periph_TIM3		|	
		RCC_APB1Periph_TIM2		|	
		0, ENABLE
	);
	
	/* APB2 clocks enable */
	RCC_APB2PeriphClockCmd(
		RCC_APB2Periph_ADC1		|
		RCC_APB2Periph_SPI5		|
		RCC_APB2Periph_TIM11	|
		RCC_APB2Periph_TIM10	|
		RCC_APB2Periph_TIM9		|
		RCC_APB2Periph_TIM1		|
		RCC_APB2Periph_SYSCFG	|
		RCC_APB2Periph_SPI4		|
		RCC_APB2Periph_SPI1		|
		RCC_APB2Periph_SDIO		|		
		RCC_APB2Periph_USART6	|		
		RCC_APB2Periph_USART1	|
		0, ENABLE
	);
	
	/* Initialise GPIOA */
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_StructInit(&GPIO_InitStructure);
	GPIO_InitStructure.GPIO_PuPd		= GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
//	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_11 | GPIO_Pin_12);		// leave USB D+/D- alone
	GPIO_InitStructure.GPIO_Pin			&= ~(GPIO_Pin_13 | GPIO_Pin_14);		// leave JTAG pins alone
	GPIO_Init(GPIOA, &GPIO_InitStructure);

	/* Initialise GPIOB */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOB, &GPIO_InitStructure);

	/* Initialise GPIOC */
	GPIO_InitStructure.GPIO_Pin			= GPIO_Pin_All;
	GPIO_Init(GPIOC, &GPIO_InitStructure);			// Board ID, AUDIO MCLK, SCLK, SDIN, OTG_FS_PowerSwitchOn
	GPIO_Init(GPIOD, &GPIO_InitStructure);			// 4 USER LEDS, AUDIO, OTG_FS
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	GPIO_Init(GPIOH, &GPIO_InitStructure);
}

void systemInit(void)
{
	/* Configure the system clock */
	SetSysClock();

	/* Configure NVIC preempt/priority groups */
	NVIC_PriorityGroupConfig(NVIC_PRIORITY_GROUPING);	// SCB->AIRCR, [10:8] = 101, 2 bits of preemption priority, 2 bits of subpriority (response priority)

	/* Clear the reset flags */
	RCC_ClearFlag();		// RCC->CSR |= RCC_CSR_RMVF;
	
	/* Enable AHB, APB1, APB2 Peripheral clocks and configure GPIOx, where x = A, B, C, D, E, H */
	EnableGPIOClk();
	
	/* Initialise SysTick counter */
	SysTick_Init();
	
	/* Configure SysTick in milliseconds time base */
	SysTick_Config(SystemCoreClock / 1000);
}


//void gpioPA1Init(void)
//{
//	PA1_PIN = IOGetByTag(IO_TAG(GPIO_PA1_PIN));
//	IOInit(PA1_PIN, OWNER_SYSTEM, 0);
//	IOConfigGPIO(PA1_PIN, IOCFG_OUT_PP);
//}

//void gpioPA1Ops(void)
//{
//	IOWrite(PA1_PIN, false);
//	delay(800);
//	IOWrite(PA1_PIN, true);
//	delay(800);	
//}

//void gpioPB8Init(void)
//{
//	PB8_PIN = IOGetByTag(IO_TAG(GPIO_PB8_PIN));
//	IOInit(PB8_PIN, OWNER_SYSTEM, 0);
//	IOConfigGPIO(PB8_PIN, IOCFG_OUT_PP);
//}

//void gpioPB8Ops(void)
//{
//	IOWrite(PB8_PIN, false);
//	delay(400);
//	IOWrite(PB8_PIN, true);
//	delay(400);	
//}

#if 0	// so far so good
	__g_ToDo = __basepriSetMemRetVal(0x10);		// basepri can be set to 0x10 only
	__g_basepri_save = __get_BASEPRI();			// __get_BASEPRI() returns 0x10
	
	__set_BASEPRI(0x00);		// reset basepri to 0x00
	__g_basepri_save2 = __get_BASEPRI();		// __get_BASEPRI() returns 0x00

	__g_ToDo2 = __basepriSetMemRetVal(0x10);		// basepri can be set to 0x10 only
	__g_basepri_save3 = __get_BASEPRI();			// __get_BASEPRI() returns 0x10

	basepri_val = 0x10;
	__basepriRestoreMem(&basepri_val);

	basepri_val = 0x00;
	__basepriRestoreMem(&basepri_val);
#endif

#if 0
	{
//		uint8_t __basepri_save = __get_BASEPRI();
		uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI();
		__g_basepri_save = __basepri_save;
		uint8_t __ToDo = __basepriSetMemRetVal(0x10);
		__g_basepri_save2 = __get_BASEPRI();
		forCnt++;
	}
#endif
	
#if 0
	for ( uint8_t __basepri_save __attribute__ ((__cleanup__(__basepriRestoreMem))) = __get_BASEPRI(), __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
//	for ( uint8_t __ToDo = __basepriSetMemRetVal(0x10); __ToDo ; __ToDo = 0 ) {
		__g_basepri_save2 = __get_BASEPRI();
		//__set_BASEPRI(0x00);
	}
	//__set_BASEPRI(0x00);
	__g_basepri_save3 = __get_BASEPRI();	// WTF?! __g_basepri_save2 should be 0x00, but it has 0x10??!!
#endif

#if 0
void clean_up(int *final_value)
{
	g_val = *final_value;
}
#endif
	
#if 0
	{
		int avar __attribute__ ((__cleanup__(clean_up))) = 1;
	}
#endif
