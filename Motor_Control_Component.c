#include "Motor_Control_Component.h"


QueueHandle_t motor_angle_queue;
QueueHandle_t motor_speed_queue;
QueueHandle_t motor_mode_queue;


void delay(){
	/******* Delay *******/
	for(volatile int i = 0U; i < 1000000; i++)
		__asm("NOP");
}

/*************** Motor speed Task ***************/
/* Used to spin the dc motor, depending on the value of input passed to motor speed queue*/
void motorSpeedTask(void* pvParameters)
{
	motor_speed_queue = (QueueHandle_t)pvParameters;
	BaseType_t status;
	motor_data temp_2;
	uint16_t mode;


	float dcCenterPoint = 0.069;
	float dc_dutyCycle = dcCenterPoint;

	float directionCoeffecient = 1.0;	// DEfaults to forward
	float modeCoeffecient = (3.0/3.0);	// Defaults to fastest speed

	dc_dutyCycle=0.06;

	while (1){

		//Receieve struct with direction and speed
		status = xQueueReceive(motor_speed_queue, (void *) &temp_2, portMAX_DELAY);
		if (status != pdPASS)
		{
			PRINTF("Motor Speed Queue Receive failed!.\r\n");
			while (1);
		}
		//Receive mode for how fast the range is
		status = xQueueReceive(motor_mode_queue, (void *) &mode, portMAX_DELAY);
		if (status != pdPASS)
		{
			PRINTF("Motor Speed Queue Receive failed!.\r\n");
			while (1);
		}



		// PWM Calculations and trigger
		// --Directions--
		if(temp_2.motor_direc==1000){
			directionCoeffecient = 1.0;
		}
		// Backward Direction
		else if (temp_2.motor_direc==2000){
			directionCoeffecient = -1.0;
		}

		// ---Speed Modes---
		// Full Speed - Re (-100 to 100)
		if(mode==2000){
			modeCoeffecient = (3.0/3.0);
		}
		// Medium Speed - Yellow (-66 to 66)
		else if(mode==1500){
			modeCoeffecient = (2.0/3.0);
		}
		// Low Speed - Green (-33 to 33)
		else if(mode==1000){
			modeCoeffecient = (1.0/3.0);
		}
		int speed;
		speed = (int)(directionCoeffecient*modeCoeffecient*(temp_2.motor_speed-1000.0f)/10.0f);


		dc_dutyCycle = (directionCoeffecient*modeCoeffecient*(temp_2.motor_speed-1000.0f)/10.0f)*(0.025f/100.0f) + dcCenterPoint;
		updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, dc_dutyCycle);
		// Trigger to spin motor
		FTM_SetSoftwareTrigger(FTM_MOTORS, true);


	}
}

/*************** Motor position Task ***************/
/*Turn Servo motor depending on the motor position*/
void motorPositionTask(void* pvParameters)
{
	motor_angle_queue = (QueueHandle_t)pvParameters;
	BaseType_t status;
	uint16_t motorPositionInput;
	float servo_dutyCycle;
	float centerPoint = 0.06975;

	// Initializing the angle to 0
	updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, centerPoint);
	FTM_SetSoftwareTrigger(FTM_MOTORS, true);

	while (1){
		status = xQueueReceive(motor_angle_queue, (void *) &motorPositionInput, portMAX_DELAY);
		if (status != pdPASS)
		{
			PRINTF("Motor Position Queue Receive failed!.\r\n");
			while (1);
		}

		// PWM Calculations and trigger
		servo_dutyCycle = (motorPositionInput/5.0f-300)*0.025f/100.0f + centerPoint;
		updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, servo_dutyCycle);
		FTM_SetSoftwareTrigger(FTM_MOTORS, true);
	}
}



void setupMotorComponent()
{
	BaseType_t status;
	setupMotorPins();

	setupDCMotor();
	setupServo();

	//Create Motor Queue speed
	motor_data motor_queue;
	motor_speed_queue = xQueueCreate(1, sizeof(motor_data));
	if (motor_speed_queue == NULL)// Error handling
	{
		PRINTF("Queue creation failed!.\r\n");
		while (1);
	}

	//Create Motor Mode Queue
	motor_mode_queue = xQueueCreate(1, sizeof(uint16_t));
	if (motor_mode_queue == NULL)// Error handling
	{
		PRINTF("Queue creation failed!.\r\n");
		while (1);
	}

	//Create Motor Task, Dc motor
	status = xTaskCreate(motorSpeedTask, "motor_speed_task", 200, (void*)motor_speed_queue, 3, NULL);
	if (status != pdPASS)// Error handling
	{
		PRINTF("Task creation failed!.\r\n");
		while (1);
	}

    /*************** Position Task ***************/
	//Create Angle Queue
	motor_angle_queue = xQueueCreate(1, sizeof(uint16_t));
	if (motor_angle_queue == NULL)
	{
		PRINTF("Queue creation failed!.\r\n");
		while (1);
	}

	//Create Position Task, Servo motor
	status = xTaskCreate(motorPositionTask, "motor_position_task", 200, (void*)motor_angle_queue, 3, NULL);
	if (status != pdPASS)// Error handling
	{
		PRINTF("Task creation failed!.\r\n");
		while (1);
	}
}

void setupMotorPins()
{
    //Configure PWM pins for DC and Servo motors
	CLOCK_EnableClock(kCLOCK_PortC);
	CLOCK_EnableClock(kCLOCK_PortA);
	PORT_SetPinMux(PORTC, 1U, kPORT_MuxAlt4);
	PORT_SetPinMux(PORTA, 6U, kPORT_MuxAlt3);

}

void setupDCMotor()
{
	//Initialize PWM for DC motor
		ftm_config_t ftmInfo;
		ftm_chnl_pwm_signal_param_t ftmParam;
		ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;

		ftmParam.chnlNumber = FTM_CHANNEL_DC_MOTOR;
		ftmParam.level = pwmLevel;
		ftmParam.dutyCyclePercent = 7;
		ftmParam.firstEdgeDelayPercent = 0U;
		ftmParam.enableComplementary = false;
		ftmParam.enableDeadtime = false;

		FTM_GetDefaultConfig(&ftmInfo);
		ftmInfo.prescale = kFTM_Prescale_Divide_128;

		FTM_Init(FTM_MOTORS, &ftmInfo);
		FTM_SetupPwm(FTM_MOTORS, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_BusClk));
		FTM_StartTimer(FTM_MOTORS, kFTM_SystemClock);

		updatePWM_dutyCycle(FTM_CHANNEL_DC_MOTOR, 0.075);
		FTM_SetSoftwareTrigger(FTM_MOTORS, true);
	//Initialize PWM for DC motor
}

void setupServo()
{
	//Initialize PWM for Servo motor
		ftm_config_t ftmInfo;
		ftm_chnl_pwm_signal_param_t ftmParam;
		ftm_pwm_level_select_t pwmLevel = kFTM_HighTrue;

		ftmParam.chnlNumber = FTM_CHANNEL_SERVO_MOTOR;
		ftmParam.level = pwmLevel;
		ftmParam.dutyCyclePercent = 7;
		ftmParam.firstEdgeDelayPercent = 0U;
		ftmParam.enableComplementary = false;
		ftmParam.enableDeadtime = false;

		FTM_GetDefaultConfig(&ftmInfo);
		ftmInfo.prescale = kFTM_Prescale_Divide_128;

		FTM_Init(FTM_MOTORS, &ftmInfo);
		FTM_SetupPwm(FTM_MOTORS, &ftmParam, 1U, kFTM_EdgeAlignedPwm, 50U, CLOCK_GetFreq(kCLOCK_BusClk));
		FTM_StartTimer(FTM_MOTORS, kFTM_SystemClock);

		updatePWM_dutyCycle(FTM_CHANNEL_SERVO_MOTOR, 0.075);
		FTM_SetSoftwareTrigger(FTM_MOTORS, true);
}

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle)
{
	uint32_t cnv, cnvFirstEdge = 0, mod;

	/* The CHANNEL_COUNT macro returns -1 if it cannot match the FTM instance */
	assert(-1 != FSL_FEATURE_FTM_CHANNEL_COUNTn(FTM_MOTORS));

	mod = FTM_MOTORS->MOD;
	if(dutyCycle == 0U)
	{
		/* Signal stays low */
		cnv = 0;
	}
	else
	{
		cnv = mod * dutyCycle;
		/* For 100% duty cycle */
		if (cnv >= mod)
		{
			cnv = mod + 1U;
		}
	}

	FTM_MOTORS->CONTROLS[channel].CnV = cnv;
}

