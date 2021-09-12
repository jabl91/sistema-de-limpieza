/*
 * Copyright 2019 NXP
 * All rights reserved.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include "pin_mux.h"
#include "clock_config.h"
#include "board.h"

/*******************************************************************************
 * Definitions
 ******************************************************************************/
#define BOARD_LED_GPIO     BOARD_LED_RED_GPIO
#define BOARD_LED_GPIO_PIN BOARD_LED_RED_PIN

/*******************************************************************************
 * Prototypes
 ******************************************************************************/

/*******************************************************************************
 * Variables
 ******************************************************************************/
volatile uint32_t g_systickCounter;

/*******************************************************************************
 * Code
 ******************************************************************************/
void SysTick_Handler(void)
{
    if (g_systickCounter != 0U)
    {
        g_systickCounter--;
    }
}

void SysTick_DelayTicks(uint32_t n)
{
    g_systickCounter = n;
    while (g_systickCounter != 0U)
    {
    }
}

/*!
 * @brief Main function
 */
int main(void)
{
    /* Board pin init */
    BOARD_InitPins();
    BOARD_InitBootClocks();

    /* Set systick reload value to generate 1ms interrupt */
    if (SysTick_Config(SystemCoreClock / 1000U))
    {
        while (1)
        {
        }
    }

    // Set the left motor to move in one direction
    GPIO_PortSet(BOARD_CTRL_DIR_ML_1_GPIO, 1u << BOARD_CTRL_DIR_ML_1_PIN);
    GPIO_PortClear(BOARD_CTRL_DIR_ML_2_GPIO, 1u << BOARD_CTRL_DIR_ML_2_PIN);

    // Set the right motor to move in one direction
    GPIO_PortClear(BOARD_CTRL_DIR_MR_1_GPIO, 1u << BOARD_CTRL_DIR_MR_1_PIN);
    GPIO_PortSet(BOARD_CTRL_DIR_MR_2_GPIO, 1u << BOARD_CTRL_DIR_MR_2_PIN);

    // Set speed of both motors to highest speed possible
    GPIO_PortSet(BOARD_CTRL_PWR_ML_GPIO, 1u << BOARD_CTRL_PWR_ML_PIN);
    GPIO_PortSet(BOARD_CTRL_PWR_MR_GPIO, 1u << BOARD_CTRL_PWR_MR_PIN);

    while (1)
    {
        /* Delay 1000 ms */
        SysTick_DelayTicks(3000U);
        GPIO_PortToggle(BOARD_LED_GPIO, 1u << BOARD_LED_GPIO_PIN);

        // Change direction of vehicle
        GPIO_PortToggle(BOARD_CTRL_DIR_ML_1_GPIO, 1u << BOARD_CTRL_DIR_ML_1_PIN);
        GPIO_PortToggle(BOARD_CTRL_DIR_ML_2_GPIO, 1u << BOARD_CTRL_DIR_ML_2_PIN);

        GPIO_PortToggle(BOARD_CTRL_DIR_MR_1_GPIO, 1u << BOARD_CTRL_DIR_MR_1_PIN);
        GPIO_PortToggle(BOARD_CTRL_DIR_MR_2_GPIO, 1u << BOARD_CTRL_DIR_MR_2_PIN);


    }
}
