#define STM32F446xx
#include "stm32f4xx.h"

void SystemClock_Config(void);
void TIM2_IRQHandler(void); // Обработчик прерывания
void Timer2_Init(void);

int main(void) {
    SystemClock_Config();  // Настраиваем тактирование
    Timer2_Init();
    while (1) {
    }
}

void SystemClock_Config(void) {
    // Настраиваем Flash (задержки ожидания)
    FLASH->ACR = FLASH_ACR_PRFTEN | FLASH_ACR_ICEN | FLASH_ACR_DCEN | FLASH_ACR_LATENCY_4WS;
    // Включаем HSE (внешний генератор 20 МГц)
    RCC->CR |= RCC_CR_HSEON;
    while (!(RCC->CR & RCC_CR_HSERDY)); // Ждем готовности HSE

    // Настраиваем PLL
    RCC->PLLCFGR = (10 << RCC_PLLCFGR_PLLM_Pos) |  // PLLM = 10
                   (150 << RCC_PLLCFGR_PLLN_Pos) | // PLLN = 150
                   (0 << RCC_PLLCFGR_PLLP_Pos) |   // PLLP = 2 (00b)
                   (1 << RCC_PLLCFGR_PLLSRC_Pos);  // Источник PLL - HSE

    // Включаем PLL
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Ждем готовности PLL

    // Настраиваем делители шин
    RCC->CFGR |= (0 << RCC_CFGR_HPRE_Pos) |   // AHB Prescaler = 1 (150 МГц)
                 (5 << RCC_CFGR_PPRE1_Pos) |  // APB1 Prescaler = 4 (37.5 МГц)
                 (0 << RCC_CFGR_PPRE2_Pos);   // APB2 Prescaler = 1 (150 МГц)

    // Переключаемся на PLL как основной источник тактирования
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Ждем переключения

    
}


// Инициализация таймера TIM2
void Timer2_Init(void) {
    // Включаем тактирование TIM2
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // Настраиваем PSC и ARR
    TIM2->PSC = 14;       // Предделитель (деление на 15)
    TIM2->ARR = 999;      // Автозагрузка (считаем до 1000)

    // Включаем прерывание по переполнению
    TIM2->DIER |= TIM_DIER_UIE;

    // Включаем таймер
    TIM2->CR1 |= TIM_CR1_CEN;

    // Разрешаем прерывание TIM2 в NVIC
    NVIC_EnableIRQ(TIM2_IRQn);
    NVIC_SetPriority(TIM2_IRQn, 1);
}

// Обработчик прерывания TIM2
void TIM2_IRQHandler(void) {
    if (TIM2->SR & TIM_SR_UIF) {  // Проверяем флаг обновления
        TIM2->SR &= ~TIM_SR_UIF;  // Сбрасываем флаг
        // Ваш код обработки прерывания (например, переключение светодиода)
    }
}
