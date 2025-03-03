#define STM32F446xx
#include "stm32f4xx.h"

#define LED_PIN 5  
#define TIMER_FREQUENCY_HZ 1000  

void SystemClock_Config(void);
void Timer_Config(void);
void GPIO_Config(void);

int main(void) {
    SystemClock_Config(); 
    GPIO_Config();        
    Timer_Config();      

    while (1) {
        
    }
}

void SystemClock_Config(void) {
    // Отключаем PLL перед настройкой
    RCC->CR &= ~RCC_CR_PLLON;
    while (RCC->CR & RCC_CR_PLLRDY);

    // PLL (HSE = 8 MHz, желаемая SYSCLK = 180 MHz)
    // f(VCO clock) = f(HSE) / M * N = 8MHz / 4 * 180 = 360 MHz
    // f(PLL output) = f(VCO clock) / P = 360 MHz / 2 = 180 MHz
    RCC->PLLCFGR = (4 << RCC_PLLCFGR_PLLM_Pos) |  // M = 4
                    (180 << RCC_PLLCFGR_PLLN_Pos) | // N = 180
                    (0 << RCC_PLLCFGR_PLLP_Pos) |  // P = 2
                    (RCC_PLLCFGR_PLLSRC_HSE);     // HSE
    
    //  HSE on
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY));
    
    // PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY));
    
    RCC->CFGR |= RCC_CFGR_HPRE_DIV1;  // AHB 180 MHz
    RCC->CFGR |= RCC_CFGR_PPRE1_DIV4; // APB1 45 MHz
    RCC->CFGR |= RCC_CFGR_PPRE2_DIV2; // APB2 90 MHz
    
    // Выбор PLL 
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}

void GPIO_Config(void) {

    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
 
    GPIOA->MODER |= (1 << (LED_PIN * 2));
    GPIOA->OTYPER &= ~(1 << LED_PIN);
    GPIOA->OSPEEDR |= (3 << (LED_PIN * 2));
}

void Timer_Config(void) {
    //  TIM2 tac
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;
    
    TIM2->PSC = (45000 - 1); // (APB1 = 45MHz, 45MHz / 45000 = 1kHz)
    TIM2->ARR = (1000 - 1);  //(1kHz / 1000 = 1Hz)
    
    TIM2->DIER |= TIM_DIER_UIE; // прерывание по переполнению
    TIM2->CR1 |= TIM_CR1_CEN;   // Включить таймер
    
    NVIC_EnableIRQ(TIM2_IRQn);  
}

void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) { // Проверяем флаг обновления
        TIM2->SR &= ~TIM_SR_UIF; // Сбрасываем флаг
        GPIOA->ODR ^= (1 << LED_PIN); // Инвертируем состояние пина
    }
}
