
#include <stdio.h>
#include "fc_tasks.h"
#include "fc_rc.h"
#include "scheduler.h"
#include "fc_core.h"
#include "common.h"
#include "rx.h"
#include "led.h"
#include "imu.h"		// including time.h
#include "gyro.h"
#include "acceleration.h"
#include "configMaster.h"
#include "runtime_config.h"
#include "led.h"
#include "pwm_output.h"
#include "maths.h"

//#define TASKS_LEDS_TESTING

#define TASK_PERIOD_HZ(hz)              (1000000 / (hz))            // units in microseconds (us)
#define TASK_PERIOD_MS(ms)              ((ms) * 1000)
#define TASK_PERIOD_US(us)              (us)

static void taskUpdateRxMain(timeUs_t currentTimeUs);
static void taskUpdateAccelerometer(timeUs_t currentTimeUs);
static void taskMotorEncoder(timeUs_t currentTimeUs);

/* Tasks initialisation */
cfTask_t cfTasks[TASK_COUNT] = {
    [TASK_SYSTEM] = {
        .taskName = "SYSTEM",
        .taskFunc = taskSystem,
//        .desiredPeriod = TASK_PERIOD_HZ(0.05),            // 1000000 / 0.05 = 20000000 us = 20000 ms = 20 s
//        .desiredPeriod = TASK_PERIOD_HZ(0.5),            // 1000000 / 0.5 = 2000000 us = 2000 ms = 2 s
//        .desiredPeriod = TASK_PERIOD_HZ(1),            // 1000000 / 1 = 1000000 us = 1000 ms = 1 s
        .desiredPeriod = TASK_PERIOD_HZ(10),            // 1000000 / 10 = 100000 us = 100 ms
        .staticPriority = TASK_PRIORITY_MEDIUM_HIGH,
    },
    
	/* desiredPeriod = 4000 us = 4 ms = 250 Hz for F450 quad */
    [TASK_GYROPID] = {
        .taskName = "PID",
        .subTaskName = "GYRO",
        .taskFunc = taskMainPidLoop,
//        .desiredPeriod = TASK_PERIOD_HZ(50),                // 1000000 / 50 = 20000 us = 20 ms
        .desiredPeriod = TASK_GYROPID_DESIRED_PERIOD,       // desiredPeriod = TASK_GYROPID_DESIRED_PERIOD = 125 us using STM32F4
        .staticPriority = TASK_PRIORITY_REALTIME,           // TASK_PRIORITY_REALTIME = 6
    },
	
	[TASK_ACCEL] = {
		.taskName = "ACCEL",
		.taskFunc = taskUpdateAccelerometer,
		.desiredPeriod = TASK_PERIOD_HZ(1000),				// 1000000 / 1000 = 1000 us, every 1ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// 3
	},
	
	[TASK_ATTITUDE] = {
		.taskName = "ATTITUDE",
		.taskFunc = taskIMUUpdateAttitude,
		.desiredPeriod = TASK_PERIOD_HZ(100),				// 1000000 / 100 = 10000 us = 10 ms
		.staticPriority = TASK_PRIORITY_MEDIUM,				// 3
	},

    [TASK_RX] = {
        .taskName = "RX",
        .checkFunc = rxUpdateCheck,
        .taskFunc = taskUpdateRxMain,
//        .desiredPeriod = TASK_PERIOD_HZ(1),            // 1000000 / 1 = 1000000 us = 1 s
        .desiredPeriod = TASK_PERIOD_HZ(50),            // 1000000 / 50 = 20000 us = 20 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },
	
    [TASK_MOTORENCODER] = {
        .taskName = "TESTMOTORENCODER",
        .taskFunc = taskMotorEncoder,
        .desiredPeriod = TASK_PERIOD_HZ(100),            // 1000000 / 100 = 10000 us = 10 ms
        .staticPriority = TASK_PRIORITY_HIGH,
    },	
    
#ifdef TASKS_LEDS_TESTING
    [TASK_LED3] = {
        .taskName = "LED3",
        .taskFunc = taskLed3,
        .desiredPeriod = TASK_PERIOD_HZ(2),             // 1000000 / 2 (hz) = 500000 us = 500 ms
//        .desiredPeriod = TASK_PERIOD_HZ(5),             // 1000000 / 5 (hz) = 200000 us = 200 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_LED4] = {
        .taskName = "LED4",
        .taskFunc = taskLed4,
        .desiredPeriod = TASK_PERIOD_HZ(1),             // 1000000 / 1 (hz) = 1000000 us = 1000 ms
//        .desiredPeriod = TASK_PERIOD_HZ(2.5),             // 1000000 / 2.5 (hz) = 400000 us = 400 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_LED5] = {
        .taskName = "LED5",
        .taskFunc = taskLed5,
        .desiredPeriod = TASK_PERIOD_HZ(0.67),             // 1000000 / 0.67 (hz) = 1500000 us = 1500 ms
//        .desiredPeriod = TASK_PERIOD_HZ(1.67),             // 1000000 / 1.67 (hz) = 600000 us = 600 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },

    [TASK_LED6] = {
        .taskName = "LED6",
        .taskFunc = taskLed6,
        .desiredPeriod = TASK_PERIOD_HZ(0.5),             // 1000000 / 0.5 (hz) = 2000000 us = 2000 ms
//        .desiredPeriod = TASK_PERIOD_HZ(1.25),             // 1000000 / 1.25 (hz) = 800000 us = 800 ms
        .staticPriority = TASK_PRIORITY_MEDIUM,
    },
#endif    
};

static void taskUpdateAccelerometer(timeUs_t currentTimeUs)
{
	UNUSED(currentTimeUs);
	
	accUpdate(currentTimeUs, &AccelerometerConfig()->accelerometerTrims);
}

static void taskUpdateRxMain(timeUs_t currentTimeUs)			// TODO: make this function static for rtos tasks assignment
{
	/* retrieve RX data */
	processRx(currentTimeUs);
	
	/* new rx data is available */
	isRXDataNew = true;
	
	/* updateRcCommands function sets rcCommand  */
	updateRcCommands();
	
	/* update LEDs */
	updateLEDs();
}

int Read_Encoder(uint8_t TIMX)
{
    int Encoder_TIM;
	switch(TIMX)
	{
		case 2:
			Encoder_TIM = (short)TIM2->CNT;
			TIM2->CNT = 0;
			break;
		
		case 3:
			Encoder_TIM = (short)TIM3->CNT;
			TIM3->CNT = 0;
			break;
		
		case 4:
			Encoder_TIM = (short)TIM4->CNT;
			TIM4->CNT = 0;
			break;
		
		default:
			Encoder_TIM = 0;
	}
	
//	printf("Encoder_TIM: %d\r\n", Encoder_TIM);
	
	return Encoder_TIM;
}

void LimitMotorPwm(int *motor)
{
	/* The maximum PWM value is 7200 setup by dcBrushedMotorInit(), we use 7100 for the upper bound */
	int Amplitude = 7100;
	
    if (*motor < -Amplitude)
		*motor = -Amplitude;	
	
	if (*motor > Amplitude)
		*motor = Amplitude;
//	printf("motor[LimitPwm]: %d\r\n", motor);
}

/**************************************************************************
Description: incremental PI controller
Parameters: Encoder value, target speed
Returns: Motor updated PWM value

Incremental discrete PID equation
pwm += Kp[e(k) - e(k-1)] + Ki * e(k) + Kd * [e(k) - 2e(k-1) + e(k-2)]
e(k) represents the current error
e(k-1) represents the previous error
pwm is the updated incremental PID value.

For speed control, the PI-only controller is utilised
pwm += Kp[e(k) - e(k-1)] + Ki * e(k)
**************************************************************************/
int Incremental_PIController(int Encoder, int Target)
{
	float Kp = 100, Ki = 100;
	static int error, updatedPWM, prev_error;
//	printf("Encoder: %d\r\n", Encoder);
	error = Encoder - Target;                		// 计算偏差
//	printf("Bias: %d\r\n", Bias);
	updatedPWM += Kp * (error - prev_error) + Ki * error;   	// 增量式PI控制器
//	printf("Pwm: %d\r\n", Pwm);
	prev_error = error;	                   			// Store previous bias
	return updatedPWM;                         			// Return PID PWM value
}


//#define DC_BRUSHED_MOTOR1_AIN1	PB13
//#define DC_BRUSHED_MOTOR1_AIN2	PB12
//#define DC_BRUSHED_MOTOR2_BIN1	PB14
//#define DC_BRUSHED_MOTOR2_BIN2	PB15

void UpdateMotorPwm(int *motorPwm1, int *motorPwm2)
{
	IO_t l_AIN1 = IOGetByTag(DCBrushedMotorConfig()->AIN1);		// PE3
	IO_t l_AIN2 = IOGetByTag(DCBrushedMotorConfig()->AIN2);		// PE4
	IO_t l_BIN1 = IOGetByTag(DCBrushedMotorConfig()->BIN1);		// PC13
	IO_t l_BIN2 = IOGetByTag(DCBrushedMotorConfig()->BIN2);		// PC15
//	volatile uint32_t *motor2PwmAddr = (volatile uint32_t *)((volatile char *)&TIM1->CCR1 + 0x4);

#if 1
//	printf("motor pwm: %d\r\n", motor);
	if (*motorPwm1 > 0) {
//		AIN2=1;
//		AIN1=0;
		printf("*motorPwm1: %d, %d\r\n", *motorPwm1, __LINE__);
//		IOWrite(DCBrushedMotorConfig()->AIN1, true);			// clear AIN1 to LOW
//		IOWrite(DCBrushedMotorConfig()->AIN2, false);			// set AIN2 to HIGH
//		GPIOB->BSRR |= 1<<29;				// set AIN1 to LOW
//		GPIOB->BSRR |= 1<<12;				// set AIN2 to HIGH
		IOHi(l_AIN2);
		IOLo(l_AIN1);
	} else {
//		AIN2=0;
//		AIN1=1;
		printf("*motorPwm1: %d, %d\r\n", *motorPwm1, __LINE__);
//		GPIOB->BSRR |= 1<<13;				// set AIN1 to HIGH
//		GPIOB->BSRR |= 1<<28;				// set AIN2 to LOW
//		IOWrite(DCBrushedMotorConfig()->AIN1, false);			// set AIN1 to HIGH
//		IOWrite(DCBrushedMotorConfig()->AIN2, true);			// clear AIN2 to LOW
		IOHi(l_AIN1);
		IOLo(l_AIN2);
	}
	
	pwmWriteDcBrushedMotor(0, ABS(*motorPwm1));	// 0 represents motor 1, write motor pwm value to motor 1 (PWMA)
//	TIM1->CCR1 = ABS(*motorPwm1);
#endif
	
#if 1
	if (*motorPwm2 > 0) {
//		IOLo(DCBrushedMotorConfig()->BIN2);
//		IOHi(DCBrushedMotorConfig()->BIN1);
		printf("*motorPwm2: %d, %d\r\n", *motorPwm2, __LINE__);
		IOHi(l_BIN2);
		IOLo(l_BIN1);
	} else {
		printf("*motorPwm2: %d, %d\r\n", *motorPwm2, __LINE__);
//		IOLo(DCBrushedMotorConfig()->BIN1);
//		IOHi(DCBrushedMotorConfig()->BIN2);
		IOHi(l_BIN1);
		IOLo(l_BIN2);
	}
		
//		printf("motorPwm: %d\r\n", motorPwm);
	pwmWriteDcBrushedMotor(1, ABS(*motorPwm2));	// 1 represents motor 2, write motor pwm value to motor 12 (PWMB)
//	*motor2PwmAddr = ABS(*motorPwm2);
#endif
}

int Target_velocity = 3;
int PWMMotor1 = 0;                 // Motor PWM value
int PWMMotor2 = 1000;                 // Motor PWM value

static void taskMotorEncoder(timeUs_t currentTimeUs)
{
//	IO_t l_AIN1 = IOGetByTag(DCBrushedMotorConfig()->AIN1);		// PE3
//	IO_t l_AIN2 = IOGetByTag(DCBrushedMotorConfig()->AIN2);		// PE4
	
//	printf("currentTimeUs: %u\r\n", currentTimeUs);
	int Encoder1, Encoder2;
	
	Encoder1 = Read_Encoder(2);		// 2: TIM2
//	Encoder2 = Read_Encoder(4);		// 4: TIM4
	
	LED6_TOGGLE;
	
	PWMMotor1 = Incremental_PIController(Encoder1, Target_velocity);
//	PWMMotor2 = Incremental_PIController(Encoder2, Target_velocity);
//	printf("Motor: %d\r\n", Motor1);
	LimitMotorPwm(&PWMMotor1);
	LimitMotorPwm(&PWMMotor2);
	UpdateMotorPwm(&PWMMotor1, &PWMMotor2);
//	IOHi(l_AIN2);
//	IOLo(l_AIN1);
//	TIM1->CCR1 = ABS(PWMMotor1);
	
//	printf("Encoder1: %d\r\n", Encoder1);
//	printf("Encoder2: %d\r\n", Encoder2);
//	printf("Motor1 pwm: %d\r\n", PWMMotor1);
//	printf("Motor2 pwm: %d\r\n", PWMMotor2);
}

void fcTasksInit(void)
{
    /* Clear RTOS queue and enable SYSTEM TASK */
    schedulerInit();

	/* Enable PID TASK including, subprocessRx, gyroUpdate, PIDController, MotorUpdate */
//	printf("gyro.targetLooptime: %u\r\n", gyro.targetLooptime);	// 1000 us
//    rescheduleTask(TASK_GYROPID, TASK_PERIOD_HZ(100));       	// JUST for testing in MATLAB, 100Hz
//    rescheduleTask(TASK_GYROPID, TASK_PERIOD_HZ(256));       	// JUST for testing in MATLAB, 256Hz
////    rescheduleTask(TASK_GYROPID, gyro.targetLooptime);       	// 1000Hz for standard ESC using PWM signals (on YQ450 quadcopter)
//    rescheduleTask(TASK_GYROPID, TASK_PERIOD_HZ(500));       	// 500Hz for standard ESC using PWM signals (on YQ450 quadcopter)
//    rescheduleTask(TASK_GYROPID, TASK_PERIOD_HZ(2000));       	// 2Khz for racing quadcopter using oneshot125, oneshot42 or multishot motor protocols
//    setTaskEnabled(TASK_GYROPID, true);

	/* Enable MOTOR ENCODER TESTING Task */
	setTaskEnabled(TASK_MOTORENCODER, true);
	
//	printf("ACC on: %d\r\n", sensors(SENSOR_ACC));						// sensors(SENSOR_ACC) = 1
//	printf("accSamplingInterval: %u\r\n", acc.accSamplingInterval);		// acc.accSamplingInterval = 1000

#if 0	
	/* Enable ACCELEROMETER TASK */
	if (sensors(SENSOR_ACC)) {
		setTaskEnabled(TASK_ACCEL, true);
		rescheduleTask(TASK_ACCEL, acc.accSamplingInterval);
	}
	
	/* Enable ATTITUDE TASK */
	setTaskEnabled(TASK_ATTITUDE, sensors(SENSOR_ACC));
	
	/* Enable RADIO RX TASK */
    setTaskEnabled(TASK_RX, true);
#endif
    
#ifdef TASKS_LEDS_TESTING
    /* Add LED3 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED3, true);

    /* Add LED4 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED4, true);

    /* Add LED5 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED5, true);

    /* Add LED6 TASK to the queue (JUST for TESTING) */
    setTaskEnabled(TASK_LED6, true);
#endif    
}
