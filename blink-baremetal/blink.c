// blink.c

// PE5 for my stm8s105 board and PD0 for my stm8s208 board

// comment out this line if stm8s208
#define STM8S105 1

#include <stdint.h>

// find all these register defs in datasheet.
#define _SFR_(mem_addr)     (*(volatile uint8_t *)(0x5000 + (mem_addr)))

// PORT D 
#define PD_ODR      _SFR_(0x0F)
#define PD_DDR      _SFR_(0x11)
#define PD_CR1      _SFR_(0x12)

// PORT E
#define PE_ODR      _SFR_(0x14)
#define PE_DDR      _SFR_(0x16)
#define PE_CR1      _SFR_(0x17)


// CLOCK (Clock divider register)
#define CLK         _SFR_(0xc6)

// PE5 for my stm8s105 board
#ifdef STM8S105 
#define LED_PIN 5
#else
#define LED_PIN 0
#endif

#define F_CPU 16000000UL //16Mhz

static inline void delay_ms(uint16_t ms) {
    uint32_t i;
    for (i = 0; i < ((F_CPU / 18000UL) * ms); i++)
        __asm__("nop");
}

void main() {
    CLK = 0x00; // switch to 16Mhz

#ifdef STM8S105
    PE_DDR |= (1 << LED_PIN); // configure PE0 as output
    PE_CR1 |= (1 << LED_PIN); // push-pull mode
    while (1) {
        /* toggle pin every 1000ms */
        PE_ODR ^= (1 << LED_PIN);
        delay_ms(1000);
    }
#else
    PD_DDR |= (1 << LED_PIN); // configure PD0 as output
    PD_CR1 |= (1 << LED_PIN); // push-pull mode
    while (1) {
        /* toggle pin every 1000ms */
        PD_ODR ^= (1 << LED_PIN);
        delay_ms(1000);
    }
#endif
}
