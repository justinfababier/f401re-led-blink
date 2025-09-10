#include "stm32f401xe.h"

#define TIMER_CLOCK_HZ  84000000    // APB1 timer clock for TIM2 (84 MHz for NUCLEO-F401RE)
#define TIMER_TICK_HZ   1000        // Timer tick frequency: 1 kHz -> 1 ms per tick
#define BLINK_PERIOD_MS 1000        // LED blink period

void GPIO_init(void) {
    // Enable clock for GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;

    // Set PA5 as general purpose output
    GPIOA->MODER &= ~(3 << (5 * 2));    // Clear mode bits for pin 5
    GPIOA->MODER |=  (1 << (5 * 2));    // Set mode = 01 (output)
}

void TIM2_init(void) {
    // Enable TIM2 clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    TIM2->PSC = (TIMER_CLOCK_HZ / TIMER_TICK_HZ) - 1;  // Set prescaler
    TIM2->ARR = BLINK_PERIOD_MS - 1;   // Set Auto-Reload Register

    TIM2->CR1 |= TIM_CR1_CEN;   // Start counting from 0 up to ARR
}

int main(void) {
    // This code is used on a NUCLEO-F401RE board
    GPIO_init();
    TIM2_init();
    
    while(1) {
        if (TIM2->SR & TIM_SR_UIF) {    // Check if UIF is set
            TIM2->SR &= ~TIM_SR_UIF;    // Clear the flag
            GPIOA->ODR ^= (1 << 5);     // Toggle on-board LED
        }
    }
}