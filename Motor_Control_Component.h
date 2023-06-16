#ifndef MOTOR_CONTROL_COMPONENT_H
#define MOTOR_CONTROL_COMPONENT_H

#include "pin_mux.h"
#include "fsl_port.h"
#include "fsl_ftm.h"
#include "fsl_debug_console.h"

#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include "Terminal_Component.h"


#define FTM_MOTORS 				FTM0
#define FTM_CHANNEL_DC_MOTOR	kFTM_Chnl_0	//Define DC motor channel
#define FTM_CHANNEL_SERVO_MOTOR		kFTM_Chnl_3//Define servo PWM channel

//extern QueueHandle_t motor_queue, angle_queue;

void setupMotorComponent();
void setupDCMotor();
void setupServo();
void setupMotorPins();

void updatePWM_dutyCycle(ftm_chnl_t channel, float dutyCycle);

void motorSpeedTask(void* pvParameters);
void motorPositionTask(void* pvParameters);

typedef struct motor_data {
	uint16_t motor_speed;
	uint16_t motor_direc;
}motor_data;

extern QueueHandle_t motor_angle_queue;
extern QueueHandle_t motor_speed_queue;
extern QueueHandle_t motor_mode_queue;

#endif /* MOTOR_CONTROL_COMPONENT_H */
