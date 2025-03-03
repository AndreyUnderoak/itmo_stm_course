#define STM32F446xx
#include "stm32f4xx.h"


#define STM32F446xx
#include "stm32f4xx.h"

#define USART2_BAUDRATE 9600
#define PCLK1_FREQ 16000000
#define USART2_BRR (PCLK1_FREQ / USART2_BAUDRATE)

volatile uint8_t received_bytes[] = {0,0};
volatile uint8_t transmit_bytes[] = {0,0};
volatile uint8_t led_status[] = {0,0,0,0,0,0,0,0,0};
volatile uint8_t received_count = 0;
volatile uint8_t data_ready = 0;

/*
   1й LED = 1й LED

    0-8 зажечь от 0 до 8 светодиодов
    10 + 1-8 вернуть информацию о светодиоде
    20 + 2 ping
    20 + 3 зажечь    все светодиоды
    20 + 4 выключить все светодиоды

*/

uint8_t lab_proto(uint8_t message);

/*
    id 1-8
    status 0/1
*/
uint8_t check_led_status(uint8_t id);

void USART2_IRQHandler(void) {
    if (USART2->SR & USART_SR_RXNE) {
        received_bytes[received_count++] = USART2->DR; 
        if (received_count == 2) {
            uint8_t message = (received_bytes[0] - '0') * 10 + (received_bytes[1] - '0');
            uint8_t out = lab_proto(message);
            transmit_bytes[0] = out / 10 + '0';
            transmit_bytes[1] = out % 10 + '0';
            
            data_ready = 1;
            received_count = 0;
            USART2->CR1 |= USART_CR1_TXEIE; 
        }
    }
    
    if ((USART2->SR & USART_SR_TXE) && data_ready==1) {
        static uint8_t transmit_count = 0;
        uint8_t buf = transmit_bytes[transmit_count++];
        USART2->DR = buf; 
        if (transmit_count == 2) {
            transmit_count = 0;
            data_ready=0;
            USART2->CR1 &= ~USART_CR1_TXEIE;
        }
    }
}



void UART2_Init(void) {
    // тактирование GPIOA
    RCC->AHB1ENR |= RCC_AHB1ENR_GPIOAEN;  
    // тактирование UART
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN; 
    
    // PA2 PA3 alt
    GPIOA->MODER |= (2 << (2 * 2)) | (2 << (3 * 2)); 
    GPIOA->AFR[0] |= (7 << (2 * 4)) | (7 << (3 * 4)); 
    
    // BRR = 16000000/9600
    USART2->BRR = 0x0683; 
    // receive transmit on
    USART2->CR1 |= USART_CR1_RE | USART_CR1_TE;
    // IRQ Receive
    USART2->CR1 |= USART_CR1_RXNEIE; 
    // USART2 on
    USART2->CR1 |= USART_CR1_UE; 
    
    NVIC_EnableIRQ(USART2_IRQn); 
}

void init_GPIO() {
    // тактирование портов C и D
    RCC->AHB1ENR |= (1 << 2) | (1 << 3);
    
    // PC4 - PC11 выходы
    for (int i = 4; i <= 11; i++) {
        GPIOC->MODER &= ~(3 << (i * 2));
        GPIOC->MODER |= (1 << (i * 2));
    }
    
    // PC13 и PD2 входы
    GPIOC->MODER &= ~(3 << (13 * 2));
    GPIOD->MODER &= ~(3 << (2 * 2));
    
}

int main(void) {
    UART2_Init();
    init_GPIO();
    for (;;) {
    }
}


uint8_t lab_proto(uint8_t message){
    uint8_t ret_val = message;
    if(message < 9){
        uint8_t i = 1;
        for(i; i <= message; ++i)
            led_switch(i, 1);
        for(i; i < 9; ++i)
            led_switch(i, 0);
    }
    else if(message > 10 && message < 19){
        ret_val = check_led_status(message - 10);
    }   
    else{
        switch (message)
        {
        case 22:
            ret_val = 22;
            break;
        
        case 23:
            for(uint8_t i = 0; i < message; ++i)
                led_switch(i, 1);
            break;
        case 24:
            for(uint8_t i = 1; i < message; ++i)
                led_switch(i, 0); 
            break;
        }
    }
    return ret_val; 
}

void led_switch(uint8_t id, uint8_t status){
    // led_status[id] = status;
    GPIOC->ODR &= ~(1 << (3+id));
    GPIOC->ODR |= status << (3+id);
}

uint8_t check_led_status(uint8_t id) {
    if (id < 1 || id > 8) {
        return 0; 
    }

    uint8_t pin = 3 + id; 

    return (GPIOC->IDR & (1 << pin)) ? 1 : 0;  
}
