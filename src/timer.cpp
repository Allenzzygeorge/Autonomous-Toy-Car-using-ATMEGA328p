#include <avr/io.h>
#include <avr/interrupt.h>
#include "timer.h"

// This volatile variable stores our system's millisecond count.
// 'volatile' is critical because it's modified by an ISR.
// 'uint32_t' allows it to count for ~49 days before overflowing.
static volatile uint32_t system_millis = 0;

// This is the "heartbeat" ISR. It runs exactly 1000 times per second.
ISR(TIMER0_OVF_vect) {
    // This preload value ensures the interrupt triggers every 1ms.
    // (16,000,000 / 64 prescaler) / 1000Hz = 250 ticks.
    // 256 - 250 = 6.
    TCNT0 = 6;
    system_millis++;
}

void timer_init(void) {
    // 1. Setup Timer0 for a 1ms overflow interrupt.
    // Use a prescaler of 64.
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // 2. Enable the Timer0 Overflow Interrupt.
    TIMSK0 = (1 << TOIE0);
    
    // 3. Enable global interrupts (if not already done in main).
    sei();
}

uint32_t millis(void) {
    uint32_t ms;
    
    // Reading a multi-byte variable that's changed by an ISR is not
    // atomic. We must disable interrupts briefly to get a clean read.
    cli(); // Disable interrupts
    ms = system_millis;
    sei(); // Re-enable interrupts
    
    return ms;
}