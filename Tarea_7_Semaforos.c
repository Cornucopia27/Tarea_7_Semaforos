/*
 * Copyright (c) 2017, NXP Semiconductor, Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * o Redistributions of source code must retain the above copyright notice, this list
 *   of conditions and the following disclaimer.
 *
 * o Redistributions in binary form must reproduce the above copyright notice, this
 *   list of conditions and the following disclaimer in the documentation and/or
 *   other materials provided with the distribution.
 *
 * o Neither the name of NXP Semiconductor, Inc. nor the names of its
 *   contributors may be used to endorse or promote products derived from this
 *   software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
 * ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
 
/**
 * @file
 * @brief   Application entry point.
 */
#include <stdio.h>
#include "board.h"
#include "peripherals.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "MK64F12.h"
#include "fsl_debug_console.h"
#include "fsl_port.h"
#include "fsl_gpio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#define SEMAPHORE_START 0
#define SEMAPHORE_MAX 10

#define SW3_PORT PORTA
#define SW2_PORT PORTC
#define SW2_PIN 6
#define SW3_PIN 4

#define BLUE_RED_LED_PORT PORTB
#define GREEN_LED_PORT PORTE
#define BLUE_LED_PIN 21
#define GREEN_LED_PIN 26

SemaphoreHandle_t Blue_semaphore;
SemaphoreHandle_t Green_semaphore;

void PORTA_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    PORT_ClearPinsInterruptFlags(SW3_PORT, 1 << SW3_PIN);
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(Blue_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void PORTC_IRQHandler()
{
    BaseType_t xHigherPriorityTaskWoken;
    PORT_ClearPinsInterruptFlags(SW2_PORT, 1 << SW2_PIN);
    xHigherPriorityTaskWoken = pdFALSE;
    xSemaphoreGiveFromISR(Green_semaphore, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void task_blue()
{
	for(;;)
	{
	    xSemaphoreTake(Blue_semaphore, portMAX_DELAY);
	    GPIO_TogglePinsOutput(GPIOB, 1 << BLUE_LED_PIN);
	}
}

void task_green()
{
	for(;;)
	{
	    uint8_t counter = (uint8_t) uxSemaphoreGetCount(Green_semaphore);
	    if(SEMAPHORE_MAX == counter)
	    {
	        GPIO_TogglePinsOutput(GPIOE, 1 << GREEN_LED_PIN);
	        while(counter != SEMAPHORE_START)
	        {
	            xSemaphoreTake(Green_semaphore, portMAX_DELAY);
	        }
	    }
	}
}

int main(void) {
  	/* Init board hardware. */
    BOARD_InitBootPins();
    BOARD_InitBootClocks();
    BOARD_InitBootPeripherals();
  	/* Init FSL debug console. */
    BOARD_InitDebugConsole();

    //Enabling the clock for the LED and Switches ports.
    CLOCK_EnableClock(kCLOCK_PortA);
    CLOCK_EnableClock(kCLOCK_PortB);
    CLOCK_EnableClock(kCLOCK_PortC);
    CLOCK_EnableClock(kCLOCK_PortE);

    //Configuration for the Leds.
    port_pin_config_t config_led =
    { kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
            kPORT_UnlockRegister, };

    //Gives the Port/Pin configuration to the blue and green pin respectively.
    PORT_SetPinConfig(BLUE_RED_LED_PORT, BLUE_LED_PIN, &config_led);
    PORT_SetPinConfig(GREEN_LED_PORT, GREEN_LED_PIN, &config_led);

    //Configuration for the switches.
    port_pin_config_t config_switch =
    { kPORT_PullDisable, kPORT_SlowSlewRate, kPORT_PassiveFilterDisable,
            kPORT_OpenDrainDisable, kPORT_LowDriveStrength, kPORT_MuxAsGpio,
            kPORT_UnlockRegister};

    //Gives the Port/Pin configuration for the SW2 and SW3 respectively.
    PORT_SetPinConfig(SW2_PORT, SW2_PIN, &config_switch);
    PORT_SetPinConfig(SW3_PORT, SW3_PIN, &config_switch);

    //Sets the interrupt on falling edge of the switches.
    PORT_SetPinInterruptConfig(SW2_PORT, SW2_PIN, kPORT_InterruptFallingEdge);
    PORT_SetPinInterruptConfig(SW3_PORT, SW3_PIN, kPORT_InterruptFallingEdge);

    //Gpio configuration as output for the leds.
    gpio_pin_config_t led_config_gpio =
    { kGPIO_DigitalOutput, 1 };

    //Sets the pins as output for the blue and green leds.
    GPIO_PinInit(GPIOB, BLUE_LED_PIN, &led_config_gpio);
    GPIO_PinInit(GPIOE, GREEN_LED_PIN, &led_config_gpio);

    //Gpio configuration as input for the switches.
    gpio_pin_config_t switch_config_gpio =
    { kGPIO_DigitalInput, 1 };

    //Sets the pins as input for the switches
    GPIO_PinInit(GPIOA, SW3_PIN, &switch_config_gpio);
    GPIO_PinInit(GPIOC, SW2_PIN, &switch_config_gpio);

    //enables the interrupts from the ports
    NVIC_EnableIRQ(PORTA_IRQn);
    NVIC_EnableIRQ(PORTC_IRQn);
    NVIC_SetPriority(PORTA_IRQn, 3);
    NVIC_SetPriority(PORTC_IRQn, 4);

    //makes the semaphore used for the blue led a binary semaphore
    Blue_semaphore = xSemaphoreCreateBinary();
    //makes the semaphore used for the green led a counting semaphore
    Green_semaphore = xSemaphoreCreateCounting(SEMAPHORE_MAX, SEMAPHORE_START);

    //creation of both the blue and green led tasks
    xTaskCreate(task_blue, "tasKB", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
    xTaskCreate(task_green, "tasKG", configMINIMAL_STACK_SIZE, NULL, configMAX_PRIORITIES-1, NULL);
    vTaskStartScheduler();

    return 0 ;
}
