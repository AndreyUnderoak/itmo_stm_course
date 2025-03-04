#define STM32F446xx
#include "stm32f4xx.h"

#define LED_PIN 5 
// частота прерывания
#define TIMER_FREQUENCY_HZ 5000

void SystemClock_INIT(void);
void Timer_INIT(void);
void GPIO_INIT(void);

int main(void) {
    SystemClock_INIT(); 
    GPIO_INIT();       
    Timer_INIT();      

    while (1) {
    }
}

void SystemClock_INIT(void) {
    // PLL off
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);

    // PLL (HSE = 8 MHz, SYSCLK = 150 MHz)
    // f(VCO clock) = f(HSE) / M * N = 8MHz / 4 * 150 = 300 MHz
    // f(PLL output) = f(VCO clock) / P = 300 MHz / 2 = 150 MHz
    RCC->PLLCFGR = (4 << RCC_PLLCFGR_PLLM_Pos) |    // M = 4
                    (150 << RCC_PLLCFGR_PLLN_Pos) | // N = 150
                    (0 << RCC_PLLCFGR_PLLP_Pos) |   // P = 2
                    (RCC_PLLCFGR_PLLSRC_HSE);       // HSE
    
    // HSE on
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    
    // PLL on
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    
    // AHB 150 MHz
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1; 
    // APB1 75 / 2 MHz 
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4;
    // APB2 75 MHz 
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; 
    
    // PLL source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void GPIO_INIT(void) {
    // GPIOA tac
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    
    // PA5 out
    GPIOA->MODER |= (1 << (LED_PIN * 2));
    GPIOA->OTYPER &= ~(1 << LED_PIN);
    GPIOA->OSPEEDR |= (3 << (LED_PIN * 2));
}

void Timer_INIT(void) {
    // tim2 tac
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    // Предделитель (APB1 = 37.5MHz, 37.5MHz / 37500 = 1kHz)
    TIM2->PSC = (37500 - 1); 
    // reload (1kHz / 5 = 5kHz)
    TIM2->ARR = (5 - 1); 
    
    // прерывание по переполнению
    TIM2->DIER |= TIM_DIER_UIE; 
    // tim2 on
    TIM2->CR1 |= TIM_CR1_CEN; 
    
    NVIC_EnableIRQ(TIM2_IRQn); 
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {
        TIM2->SR &= ~TIM_SR_UIF; 
        GPIOA->ODR ^= (1 << LED_PIN);
    }
}
