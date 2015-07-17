#pragma once

#include <stdint.h>

// Board support
//
#define LED_GREEN       1


extern int  board_address;

void board_init(void);
void board_set_leds(uint8_t leds);

uint16_t board_get_hvmon(void);
