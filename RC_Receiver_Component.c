
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "timers.h"

/* Freescale includes. */
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "RC_Receiver_Component.h"
#include "LED_Component.h"
#include "Motor_Control_Component.h"


#define BOARD_LED_GPIO_D GPIOD

#define BOARD_LED_GPIO_PIN_R 1


#define BOARD_LED_GPIO_C GPIOC
#define BOARD_LED_GPIO_PIN_B 8
#define BOARD_LED_GPIO_PIN_G 9

/**
 * Struct carries RC values
 * Struct size = 2 Bytes * 9 uint16 = 18 Bytes
 */
typedef struct {
uint16_t header;
uint16_t ch1;
uint16_t ch2;
uint16_t ch3;
uint16_t ch4;
uint16_t ch5;
uint16_t ch6;
uint16_t ch7;
uint16_t ch8;
} RC_Values;

SemaphoreHandle_t rc_hold_semaphore;
TaskHandle_t rc_task_handle;

extern QueueHandle_t motor_angle_queue;
extern QueueHandle_t motor_speed_queue;
extern QueueHandle_t motor_mode_queue;


 /*************** RC Task ***************/
void rcTask(void* pvParameters){
	BaseType_t status;

	RC_Values rc_values;
	uint8_t* ptr = (uint8_t*) &rc_values;

	while (1)
	{
		UART_ReadBlocking(UART1, ptr, 1);
		if(*ptr != 0x20)
				continue;
		UART_ReadBlocking(UART1, &ptr[1], sizeof(rc_values) - 1);

		if(rc_values.header == 0x4020)
			{


			status = xQueueSendToBack(led_queue, (void*) &rc_values.ch6, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}

			status = xQueueSendToBack(motor_mode_queue, (void*) &rc_values.ch6, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}

			status = xQueueSendToBack(motor_angle_queue, (void*) &rc_values.ch1, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}

				motor_data temp;
				temp.motor_speed=rc_values.ch3;
				temp.motor_direc=rc_values.ch7;


			status = xQueueSendToBack(motor_speed_queue, (motor_data*)&temp, portMAX_DELAY);
				if (status != pdPASS)
				{
					PRINTF("Queue Send failed!.\r\n");
					while (1);
				}


			}
		
	}
}


void setupRCReceiverComponent()
{
	BaseType_t status;
	setupRCPins();

	setupUART_RC();
	status = xTaskCreate(rcTask, "RC_Task", 200,(void*)led_queue , 3, NULL);
		if (status != pdPASS)
			{
				PRINTF("Task creation failed!.\r\n");
				while (1);
			}

   
	

}

void setupRCPins()
{
	//Configure RC pins
	BOARD_InitBootPins();
	BOARD_InitBootClocks();
	BOARD_InitDebugConsole();
}

void setupUART_RC()
{
	uart_config_t config;
	UART_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.enableTx = false;
	config.enableRx = true;
	UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));
}
