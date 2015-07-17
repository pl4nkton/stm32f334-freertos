#include "board.h"
#include "stm32f30x.h"

void board_set_leds(uint8_t leds)
{
#ifdef BOARD_NUCLEO
    // STM32F334 Nucleo
    //
    GPIOA->BSRR = (~(leds & 1)) << 21;  // clear leds
    GPIOA->BSRR =   (leds & 1)  << 5;  // set leds
#endif
}


void board_init(void)
{
    // Enable the compensation cell and GPIO clocks
    //
    //SYSCFG->CMPCR = SYSCFG_CMPCR_CMP_PD; //FIXME
    RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
//    RCC->AHBENR |= RCC_AHBENR_GPIOBEN;
//    RCC->AHBENR |= RCC_AHBENR_GPIOCEN;
//    RCC->AHBENR |= RCC_AHBENR_GPIODEN;

#ifdef BOARD_NUCLEO
    // PA5  LED_GREEN
    //
    GPIO_Init(GPIOA, &(GPIO_InitTypeDef) {
        .GPIO_Pin   = GPIO_Pin_5,
        .GPIO_Mode  = GPIO_Mode_OUT,
        .GPIO_Speed = GPIO_Speed_2MHz,
        .GPIO_OType = GPIO_OType_PP
    });
#else
#endif
}

