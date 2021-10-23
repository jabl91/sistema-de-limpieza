/*
 * Copyright (c) 2015, Freescale Semiconductor, Inc.
 * Copyright 2016-2017 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

/* FreeRTOS kernel includes. */
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

#include "fsl_uart_freertos.h"
#include "fsl_uart.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
/* UART instance and clock */
#define DEMO_UART            UART3
#define DEMO_UART_CLKSRC     UART3_CLK_SRC
#define DEMO_UART_CLK_FREQ   CLOCK_GetFreq(UART3_CLK_SRC)
#define DEMO_UART_RX_TX_IRQn UART3_RX_TX_IRQn
/* Task priorities. */
#define uart_task_PRIORITY (configMAX_PRIORITIES - 1)
#define motors_task_PRIORITY (configMAX_PRIORITIES - 2)
#define ultrasonic_task_PRIORITY (configMAX_PRIORITIES - 3)

/*******************************************************************************
 * Prototypes
 ******************************************************************************/
static void uart_task(void *pvParameters);
static void motors_task(void *pvParameters);
static void ultrasonic_task(void *pvParameters);

/*******************************************************************************
 * Variables
 ******************************************************************************/
char *to_send               = "FreeRTOS UART driver example!\r\n";
char *send_ring_overrun     = "\r\nRing buffer overrun!\r\n";
char *send_hardware_overrun = "\r\nHardware buffer overrun!\r\n";
uint8_t background_buffer[32];
uint8_t recv_buffer[4];

uart_rtos_handle_t handle;
struct _uart_handle t_handle;
volatile char directioncontrol = 0;


uart_rtos_config_t uart_config = {
    .baudrate    = 9600,
    .parity      = kUART_ParityDisabled,
    .stopbits    = kUART_OneStopBit,
    .buffer      = background_buffer,
    .buffer_size = sizeof(background_buffer),
};

/*******************************************************************************
 * Code
 ******************************************************************************/
/*!
 * @brief Application entry point.
 */
int main(void)
{
    /* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    NVIC_SetPriority(DEMO_UART_RX_TX_IRQn, 5);
    if (xTaskCreate(uart_task, "Uart_task", configMINIMAL_STACK_SIZE + 100, NULL, uart_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1);
    }
    if (xTaskCreate(motors_task, "Motors_task", configMINIMAL_STACK_SIZE + 100, NULL, motors_task_PRIORITY, NULL) != pdPASS)
    {
        PRINTF("Task creation failed!.\r\n");
        while (1);
    }
    if (xTaskCreate(ultrasonic_task, "Ultrasonic_task", configMINIMAL_STACK_SIZE +100, NULL, ultrasonic_task_PRIORITY, NULL)!= pdPASS){
    	PRINTF("Task creation failed !\r\n");
    	while (1);
    }
    vTaskStartScheduler();
    for (;;)
        ;
}

/*!
 * @brief Task responsible for motors control.
 */
static void motors_task(void *pvParameters)
{

	TickType_t xLastWakeTime;

    // Set the left motor to move in one direction
    GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_ML_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_1_PIN);
    GPIO_PortClear(BOARD_INITPINS_CTRL_DIR_ML_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_2_PIN);

    // Set the right motor to move in one direction
    GPIO_PortClear(BOARD_INITPINS_CTRL_DIR_MR_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_1_PIN);
    GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_MR_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_2_PIN);

    // Set speed of both motors to highest speed possible
    GPIO_PortSet(BOARD_INITPINS_CTRL_PWR_ML_GPIO, 1u << BOARD_INITPINS_CTRL_PWR_ML_PIN);
    GPIO_PortSet(BOARD_INITPINS_CTRL_PWR_MR_GPIO, 1u << BOARD_INITPINS_CTRL_PWR_MR_PIN);

    xLastWakeTime = xTaskGetTickCount ();
    while (1)
    {
        /* Delay 300 ticks == 600 * 5 ms == 3000 ms == 3 seconds */
    	vTaskDelayUntil( &xLastWakeTime, 600U );

    	switch(directioncontrol){
    	    case 119: //119==ASCCI w
    	    	// Forward Direction
				// Set the left motor to move in one direction
				GPIO_PortClear(BOARD_INITPINS_CTRL_DIR_ML_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_1_PIN);
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_ML_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_2_PIN);

				// Set the right motor to move in one direction
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_MR_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_1_PIN);
				GPIO_PortClear(BOARD_INITPINS_CTRL_DIR_MR_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_2_PIN);

			break;
    	    case 115: //115 ==ASCCI s
    	    	// Reverse Direction
				// Set the left motor to move in one direction
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_ML_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_1_PIN);
				GPIO_PortClear(BOARD_INITPINS_CTRL_DIR_ML_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_2_PIN);

				// Set the right motor to move in one direction
				GPIO_PortClear(BOARD_INITPINS_CTRL_DIR_MR_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_1_PIN);
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_MR_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_2_PIN);
    	    	break;
    	    case 98:
			default:
				// Brake
				// Set the left motor to move in one direction
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_ML_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_1_PIN);
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_ML_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_ML_2_PIN);

				// Set the right motor to move in one direction
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_MR_1_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_1_PIN);
				GPIO_PortSet(BOARD_INITPINS_CTRL_DIR_MR_2_GPIO, 1u << BOARD_INITPINS_CTRL_DIR_MR_2_PIN);
				break;
    		}
    }


    vTaskSuspend(NULL);
}

static void ultrasonic_task(void *pvParameters){
	TickType_t xLastWakeTime;

	GPIO_PortClear(BOARD_INITPINS_SNS_TRIGGER_GPIO, 1u << BOARD_INITPINS_SNS_TRIGGER_PIN);


	xLastWakeTime = xTaskGetTickCount ();
	while(1){
		/* Delay 200 ticks == 200 * 5 ms == 1000 ms == 1 seconds */
		vTaskDelayUntil( &xLastWakeTime, 200U );

		GPIO_PortSet(BOARD_INITPINS_SNS_TRIGGER_GPIO, 1u << BOARD_INITPINS_SNS_TRIGGER_PIN);
		for	(unsigned int x=0; x<120;x++) {

		}
		GPIO_PortClear(BOARD_INITPINS_SNS_TRIGGER_GPIO, 1u << BOARD_INITPINS_SNS_TRIGGER_PIN);
	}



}

/*!
 * @brief Task responsible for loopback.
 */
static void uart_task(void *pvParameters)
{
    int error;
    size_t n = 0;

    uart_config.srcclk = DEMO_UART_CLK_FREQ;
    uart_config.base   = DEMO_UART;

    if (kStatus_Success != UART_RTOS_Init(&handle, &t_handle, &uart_config))
    {
        vTaskSuspend(NULL);
    }

    /* Send introduction message. */
    if (kStatus_Success != UART_RTOS_Send(&handle, (uint8_t *)to_send, strlen(to_send)))
    {
        vTaskSuspend(NULL);
    }

    /* Receive user input and send it back to terminal. */
    do
    {
        error = UART_RTOS_Receive(&handle, recv_buffer, sizeof(recv_buffer), &n);
        if (error == kStatus_UART_RxHardwareOverrun)
        {
            /* Notify about hardware buffer overrun */
            if (kStatus_Success !=
                UART_RTOS_Send(&handle, (uint8_t *)send_hardware_overrun, strlen(send_hardware_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (error == kStatus_UART_RxRingBufferOverrun)
        {
            /* Notify about ring buffer overrun */
            if (kStatus_Success != UART_RTOS_Send(&handle, (uint8_t *)send_ring_overrun, strlen(send_ring_overrun)))
            {
                vTaskSuspend(NULL);
            }
        }
        if (n > 0)
        {
        	directioncontrol = recv_buffer[3];
            /* send back the received data */
            UART_RTOS_Send(&handle, recv_buffer, n);
        }
    } while (kStatus_Success == error);

    UART_RTOS_Deinit(&handle);
    vTaskSuspend(NULL);
}
