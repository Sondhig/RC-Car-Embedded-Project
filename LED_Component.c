
#include "fsl_device_registers.h"
#include "fsl_debug_console.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"
#include "fsl_uart.h"
#include "queue.h"
#include "LED_Component.h"
/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_GPIO_D GPIOD

#define BOARD_LED_GPIO_PIN_R 1


#define BOARD_LED_GPIO_C GPIOC
#define BOARD_LED_GPIO_PIN_B 8
#define BOARD_LED_GPIO_PIN_G 9

gpio_pin_config_t led_config = {
        kGPIO_DigitalOutput,
        0,
    };

QueueHandle_t led_queue;


 /*************** LED Task ***************/
 /* Turns LED color to RGB depending on value from LED Queue*/
void ledTask(void* pvParameters){
	BaseType_t status;
	int Led_speed=0;

	while(1){
		// Receive the value of LED_Speed from the LED_Queue, integer value
		status = xQueueReceive(led_queue, (void *) &Led_speed, portMAX_DELAY);
		if (status != pdPASS)
		{
			PRINTF("Queue Receive failed!.\r\n");
			while (1);
		}

		
		//Toggle Green LED
		if(Led_speed==1000){
			GPIO_PortClear(BOARD_LED_GPIO_C, 1u << BOARD_LED_GPIO_PIN_B);
			GPIO_PortClear(BOARD_LED_GPIO_D, 1u << BOARD_LED_GPIO_PIN_R);
			GPIO_PortToggle(BOARD_LED_GPIO_C, 1u << BOARD_LED_GPIO_PIN_G);
			//printf("G\t");
		}
		//Toggle Blue LED
		else if(Led_speed==1500){
			GPIO_PortClear(BOARD_LED_GPIO_D, 1u << BOARD_LED_GPIO_PIN_R);
			GPIO_PortClear(BOARD_LED_GPIO_C, 1u << BOARD_LED_GPIO_PIN_G);
			GPIO_PortToggle(BOARD_LED_GPIO_C, 1u << BOARD_LED_GPIO_PIN_B);
			//printf("B\t");
		}
		//Toggle Red LED
		else if(Led_speed==2000){
			GPIO_PortClear(BOARD_LED_GPIO_C, 1u << BOARD_LED_GPIO_PIN_B);
			GPIO_PortClear(BOARD_LED_GPIO_C, 1u << BOARD_LED_GPIO_PIN_G);
			GPIO_PortToggle(BOARD_LED_GPIO_D, 1u << BOARD_LED_GPIO_PIN_R);
			//printf("R\t");
		}
	}

}

void setupLEDComponent()
{
	setupLEDPins();
	setupLEDs();

	BaseType_t status;
   
	//Create LED Queue
	led_queue = xQueueCreate(1, sizeof(uint16_t));
	//Create LED Task
	status = xTaskCreate(ledTask, "LED_task", 200, (void*)led_queue, 2, NULL); // First Parameters is the function
	if (status != pdPASS)
	{
	PRINTF("Task creation failed!.\r\n");
	while (1);
	}

}
//Initilize Led Pins
void setupLEDPins()
{
	GPIO_PinInit(BOARD_LED_GPIO_C, BOARD_LED_GPIO_PIN_G, &led_config);
	GPIO_PinInit(BOARD_LED_GPIO_D, BOARD_LED_GPIO_PIN_R, &led_config);
	GPIO_PinInit(BOARD_LED_GPIO_C, BOARD_LED_GPIO_PIN_B, &led_config);
}

void setupLEDs()
{
	//Initialize PWM for the LEDs
}
