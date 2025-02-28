#define STM32F446xx
#include "stm32f4xx.h"

// #include <stdio.h>

volatile uint8_t received_bytes[] = {0,0};
volatile uint8_t transmit_bytes[] = {0,0};
volatile uint8_t byte_index = 0;
volatile uint8_t led_status[] = {0,0,0,0,0,0,0,0};


/*
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
void led_switch(uint8_t id, uint8_t status);

void USART2_IRQHandler(void) {
    // receive
    // if (USART2->SR & USART_SR_RXNE) { 
    //     received_byte = USART2->DR; 
    //     received_byte = received_byte - '0';
    //     transmit_byte = '0' + lab_proto(received_byte); 
    //     USART2->DR = transmit_byte;   
    //     // Ожидание отправки
    //     while (!(USART2->SR & USART_SR_TXE)); 
    // }

    if (USART2->SR & USART_SR_RXNE) {  // Прерывание по приему
        received_bytes[byte_index] = USART2->DR;   // Читаем принятый байт
        byte_index++;
        
        if (byte_index >= 2) {  // Если приняли 2 байта
            
            uint8_t message = (received_bytes[0] - '0') * 10 + (received_bytes[1] - '0');

            uint8_t out = lab_proto(message);

            transmit_bytes[0] = out / 10 + '0';
            transmit_bytes[1] = out % 10 + '0';
            byte_index = 0;

            for (uint8_t i = 0; i < 2; ++i) {
                USART2->DR = transmit_bytes[i];
                while (!(USART2->SR & USART_SR_TXE)); // Ждем, пока буфер передатчика освободится
                }
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

int main(void) {
    UART2_Init();
    for (;;) {
    }
}


uint8_t lab_proto(uint8_t message){
    uint8_t ret_val = message;
    if(message < 9){
        uint8_t i = 0;
        for(i; i < message; ++i)
            led_switch(i, 1);
        for(i; i < 9; ++i)
            led_switch(i, 0);
    }
    else if(message > 10 && message < 19)
        ret_val = led_status[message - 10];
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
            for(uint8_t i = 0; i < message; ++i)
                led_switch(i, 0); 
            break;
        }
    }
    return ret_val; 
}

void led_switch(uint8_t id, uint8_t status){
    led_status[id] = status;
}