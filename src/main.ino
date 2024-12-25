
#include <stdint.h>
#include <string.h>
#include <stm32l0xx.h>

// GPIO and UART Definitions
#define LED_PIN                 3       
#define LED_PORT                GPIOB
#define LED_PORT_CLK_ENABLE()   (RCC->IOPENR |= RCC_IOPENR_GPIOBEN)
#define UART_TX_PIN             2       
#define UART_RX_PIN             15      

void SystemClock_Config(void);
void UART2_Init(void);
void GPIO_Init(void);
void UART2_Transmit(char *data);
void delay_ms(uint32_t ms);

int main(void)
{
    SystemClock_Config();  // Set system clock
    GPIO_Init();           // Configure GPIO
    UART2_Init();          // Configure UART

    UART2_Transmit("Hello, Wokwi!\n");

    while (1)
    {
        LED_PORT->ODR ^= (1 << LED_PIN);  // Toggle LED
        delay_ms(500);                    // Delay 500ms
    }
}

// ---------------------------------
// GPIO Initialization
// ---------------------------------
void GPIO_Init(void)
{
    // Enable GPIOB and GPIOA Clock
    RCC->IOPENR |= RCC_IOPENR_GPIOBEN;  // Enable GPIOB Clock
    RCC->IOPENR |= RCC_IOPENR_GPIOAEN;  // Enable GPIOA Clock

    // Configure PB3 as Output (LED)
    LED_PORT->MODER &= ~(0x3 << (LED_PIN * 2));  // Clear mode
    LED_PORT->MODER |= (0x1 << (LED_PIN * 2));   // Set PB3 as General Purpose Output
    LED_PORT->OTYPER &= ~(1 << LED_PIN);         // Push-pull
    LED_PORT->OSPEEDR |= (0x3 << (LED_PIN * 2)); // High speed
    LED_PORT->PUPDR &= ~(0x3 << (LED_PIN * 2));  // No pull-up/pull-down

    // Configure PA2 (TX) and PA15 (RX) for USART2 Alternate Function
    GPIOA->MODER &= ~((0x3 << (UART_TX_PIN * 2)) | (0x3 << (UART_RX_PIN * 2)));
    GPIOA->MODER |= (0x2 << (UART_TX_PIN * 2)) | (0x2 << (UART_RX_PIN * 2));  // AF Mode

    GPIOA->AFR[0] |= (0x4 << (UART_TX_PIN * 4)); // PA2 -> AF4 (USART2_TX)
    GPIOA->AFR[1] |= (0x4 << ((UART_RX_PIN - 8) * 4)); // PA15 -> AF4 (USART2_RX)
}

// ---------------------------------
// UART2 Initialization
// ---------------------------------
void UART2_Init(void)
{
    // Enable USART2 Clock
    RCC->APB1ENR |= RCC_APB1ENR_USART2EN;

    // Configure USART2: 115200 baud, 8 data bits, 1 stop bit, no parity
    USART2->BRR = SystemCoreClock / 115200;  // Set baud rate
    USART2->CR1 = USART_CR1_TE | USART_CR1_RE;  // Enable TX and RX
    USART2->CR1 |= USART_CR1_UE;  // Enable USART2
}

// ---------------------------------
// UART2 Transmit
// ---------------------------------
void UART2_Transmit(char *data)
{
    while (*data)
    {
        while (!(USART2->ISR & USART_ISR_TXE)); // Wait until TX buffer is empty
        USART2->TDR = *data++;
    }
    while (!(USART2->ISR & USART_ISR_TC)); // Wait until transmission is complete
}

// ---------------------------------
// Simple Millisecond Delay
// ---------------------------------
void delay_ms(uint32_t ms)
{
    for (uint32_t i = 0; i < ms * 1000; i++)
    {
        __asm__("nop");
    }
}

void SystemClock_Config(void)
{
    // Enable HSI (16 MHz High-Speed Internal Oscillator)
    RCC->CR |= RCC_CR_HSION;
    while (!(RCC->CR & RCC_CR_HSIRDY)); // Wait until HSI is ready

    // Configure PLL: HSI -> PLLMUL4 -> PLLDIV2 -> SYSCLK (32 MHz)
    RCC->CFGR |= (RCC_CFGR_PLLMUL4 | RCC_CFGR_PLLDIV2);
    RCC->CR |= RCC_CR_PLLON;
    while (!(RCC->CR & RCC_CR_PLLRDY)); // Wait until PLL is ready

    // Select PLL as System Clock Source
    RCC->CFGR |= RCC_CFGR_SW_PLL;
    while ((RCC->CFGR & RCC_CFGR_SWS) != RCC_CFGR_SWS_PLL); // Wait for PLL to be used as the system clock
}

