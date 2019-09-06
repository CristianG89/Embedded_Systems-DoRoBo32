/**
 * DoroboLib32 Sammel-Initialisierung
 *
 * Initialisierung für die DoroboLib32 und die gebräuchlichsten Module.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#include <errno.h>
#include "dorobo32.h"

#include "stm32f0xx_hal.h"
#include "stdlib.h"


// ***************************************************************************

// Die main()-Funktion aus der CubeMX-generierten main() muss in
// init() abgespeichert und hinzugelinkt werden.
extern void init();

void dorobo_init(void)
{
    init();
    digital_init();
}

// ***************************************************************************

void delay(uint32_t duration_ms)
{
    HAL_Delay(duration_ms);
}

// ***************************************************************************

void led_green(enum DD_PINLEVEL_E level)
{
    HAL_GPIO_WritePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin, level);
}

void led_green_toggle(void)
{
    HAL_GPIO_TogglePin(LED_GREEN_GPIO_Port, LED_GREEN_Pin);
}

void led_red(enum DD_PINLEVEL_E level)
{
    HAL_GPIO_WritePin(LED_RED_GPIO_Port, LED_RED_Pin, level);
}

void led_red_toggle(void)
{
    HAL_GPIO_TogglePin(LED_RED_GPIO_Port, LED_RED_Pin);
}

// ***************************************************************************

void _exit(int status)
{
    led_red(DD_LED_OFF);
    led_green(DD_LED_ON);
    while (1)
    {
        HAL_Delay(100);
        led_green_toggle();
        if (status != 0)
        {
            led_red_toggle();
        }
        HAL_Delay(500);
        led_green_toggle();
        if (status != 0)
        {
            led_red_toggle();
        }
    }
}

// ***************************************************************************

caddr_t _sbrk(int incr)
{
    extern char _ebss;   ///< symbol defined by the linker
    extern char _estack; ///< symbol defined by the linker
    static char *heap_end;
    char *prev_heap_end;

    if (heap_end == 0)
    {
        heap_end = &_ebss;
    }
    prev_heap_end = heap_end;

    // determine the maximum address available for the Heap
    // (1) Start with the current stack pointer
    //     This heap end address becomes a problem if
    //     the stack grows further - therefore we allow for
    //     some additional room.
    char *heapMax = (char *) __get_MSP() - 128;
    if ( (char *) __get_MSP() < prev_heap_end )
    {
        // FreeRTOS multitasking has been started, because
        // the Stack usually lies in an BSS area managed
        // by FreeRTOS. Therefore the maximum heap end is
        // the end of RAM (symbol defined by the linker).
    	heapMax = &_estack; // this symbol is defined at the end of RAM
    }

    if (heap_end + incr > heapMax)
    {
        //_write (STDERR_FILENO, "Heap and heapMax collision\n", 25);
        errno = ENOMEM;
        return (caddr_t) -1;
        // Alternative implementation: exit(-1) to stop the system
    }

    heap_end += incr;
    return (caddr_t) prev_heap_end;
}

