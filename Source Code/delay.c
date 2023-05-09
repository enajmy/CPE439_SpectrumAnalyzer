/* ---------- delay.c ---------- */
#include "main.h"
#include "delay.h"

/* Configure SysTick Timer for use with delay_us function. This will break
 * break compatibility with HAL_delay() by disabling interrupts to allow for
 * shorter delay timing.
 */
void SysTick_Init(void){
    SysTick->CTRL |= (SysTick_CTRL_ENABLE_Msk |	       // enable SysTick Timer
                      SysTick_CTRL_CLKSOURCE_Msk);     // select CPU clock
    SysTick->CTRL &= ~(SysTick_CTRL_TICKINT_Msk);      // disable interrupt,
                                                       // breaks HAL delay function
}

/* Delay function using the SysTick timer to count CPU clock cycles for more
 * precise delay timing. Passing a time of 0 will cause an error and result
 * in the maximum delay. Short delays are limited by the clock speed and will
 * often result in longer delay times than specified. @ 4MHz, a delay of 1us
 * will result in a delay of 10-15 us.
 */
void delay_us(const uint32_t time_us) {
    // set the counts for the specified delay
    SysTick->LOAD = (uint32_t)((time_us * (SystemCoreClock / 1000000)) - 1);
    SysTick->VAL = 0;                                      // clear the timer count
    SysTick->CTRL &= ~(SysTick_CTRL_COUNTFLAG_Msk);        // clear the count flag
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk)); // wait for the flag
}

