#define STM32F446xx
#include "stm32f4xx.h"
#include <math.h>


// ******* GPIO *******

typedef enum
{
    INPUT_MODE = 0x00,  ///< input mode
    OUTPUT_MODE = 0x01, ///< output mode
    AF_MODE = 0x02,     ///< alternative function mode
    ANALOG_MODE = 0x03  ///< analog mode
} GPIOMode;


#define MAX_PIN_Msk (0x0FUL)
#define MODER_Msk (0x03UL)

inline void gpio_mode(GPIO_TypeDef *gpio, uint32_t pin, const GPIOMode mode);

// ******* GPIO END *******

// ******* RCC *******
#define PLLM_DIV (4)
#define PLLN_FAC (150)
void rcc_config(void);
// ******* RCC END ******


// ******* TIM *******
#define MAIN_CLK (75000000)   // Hz
#define TIMER_CLK (100000)    // Hz
#define TIMER_INT_CLK (50000) // Hz

#define DAC_GPIO GPIOA
#define DAC_PIN (4)

#define SIN_FREQ (1000.0f) // Hz
#define SIN_AMPL (1.0f)    // V
#define SIN_BIAS (1.0f)    // V

#define VREF (3.3f) // V
#define DAC_MAX_VAL (0xFFF + 1)

#define PHASE_STEP (2.0f * M_PI * SIN_FREQ / (float)TIMER_INT_CLK)

volatile float cur_phase = 0.0f;

#define TIM TIM6
// #define TIM_Handler TIM6_DAC_IRQHandler
// #define TIM_IRQ TIM6_DAC_IRQn

void dac_init(void);
// ******* TIM END *******

int main(void)
{
#if (__FPU_PRESENT == 1) && (__FPU_USED == 1)
    SCB->CPACR |= ((3UL << 10 * 2) | (3UL << 11 * 2)); /* set CP10 and CP11 Full Access */
#endif

    rcc_config();
    dac_init();

    /* Loop forever */
    for (;;)
    {
    }
}


inline void gpio_mode(GPIO_TypeDef *gpio, uint32_t pin, const GPIOMode mode)
{
    uint32_t shift = (pin & MAX_PIN_Msk) << 1UL;
    gpio->MODER &= ~(MODER_Msk << shift);
    gpio->MODER |= ((uint32_t)mode & MODER_Msk) << shift;
}


void rcc_config(void)
{
    // HSE - 8 MHz
    // PLL input - 8MHz / 4 -> 2 MHz
    // PLL VCO - 2 MHz * 150 -> 300 MHz
    // PLL output - 300 MHz / 2 -> 150 MHz
    // AHB - 150 MHz / 1 -> 150 MHz
    // APB2 - 150 MHz / 2 -> 75 MHz

    // HSE
    RCC->CR |= RCC_CR_HSEON;
    while ((RCC->CR & RCC_CR_HSERDY) == 0);

    uint32_t reg = RCC->CFGR;
    reg &= ~(RCC_CFGR_HPRE | RCC_CFGR_PPRE1 | RCC_CFGR_PPRE2);
    reg |= RCC_CFGR_HPRE_DIV1 | RCC_CFGR_PPRE1_DIV4 | RCC_CFGR_PPRE2_DIV2;
    RCC->CFGR = reg;

    FLASH->ACR &= ~FLASH_ACR_LATENCY; 
    FLASH->ACR |= FLASH_ACR_LATENCY_4WS;

    // PLL
    reg = RCC->PLLCFGR;
    reg &= ~(RCC_PLLCFGR_PLLM | RCC_PLLCFGR_PLLN | RCC_PLLCFGR_PLLP);
    reg |= (PLLM_DIV << RCC_PLLCFGR_PLLM_Pos);
    reg |= (PLLN_FAC << RCC_PLLCFGR_PLLN_Pos);
    // PLLP = 0
    reg |= RCC_PLLCFGR_PLLSRC_HSE;
    RCC->PLLCFGR = reg;

    RCC->CR |= RCC_CR_PLLON;


    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL);
}


// ******* TIM *******
void dac_init(void)
{

    // init dac
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;
    gpio_mode(DAC_GPIO, DAC_PIN, ANALOG_MODE);

    RCC->APB1ENR |= RCC_APB1ENR_DACEN;

    DAC1->CR |= DAC_CR_EN1;

    // init timer
    RCC->APB1ENR |= RCC_APB1ENR_TIM6EN;

    // Fev = 5 kHz
    // Fin = 75 MHz
    // Fcnt = 100'000 Hz
    // PSC = Fin / Fcnt - 1 = 749
    // ARR = Fcnt / Fev - 1 = 9
    TIM->PSC = MAIN_CLK / TIMER_CLK - 1;
    TIM->ARR = TIMER_CLK / TIMER_INT_CLK - 1;
    TIM->DIER |= TIM_DIER_UIE;

    NVIC_EnableIRQ(TIM6_DAC_IRQn);

    TIM->CR1 |= TIM_CR1_CEN;
}

volatile float new_val = 0;

void TIM6_DAC_IRQHandler(void)
{
    if (TIM->SR & TIM_SR_UIF)
    {
        TIM->SR &= ~TIM_SR_UIF;

        float new_vlt = SIN_AMPL * sinf(cur_phase) + SIN_BIAS;
        DAC1->DHR12R1 = (uint16_t)(new_vlt / VREF * DAC_MAX_VAL);

        cur_phase += PHASE_STEP;
        if (cur_phase >= 2.0f * M_PI)
            cur_phase -= 2.0f * M_PI;
    }
}
// ******* TIM END *******
