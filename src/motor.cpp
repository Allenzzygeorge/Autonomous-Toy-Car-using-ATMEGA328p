#include <avr/io.h>
#include "motor.h" // Include the header file

// --- Private Pin Defines ---
// These are only used within this motor module.
#define ENA_PIN     PD6 // (D6), OC0A for Timer0 PWM speed control
#define IN1_PIN     PD2 // (D2)
#define IN2_PIN     PB0 // (D8)

// --- Function Implementations ---

void motor_init(void) {
    // 1. Set all motor control pins as outputs
    DDRD |= (1 << ENA_PIN) | (1 << IN1_PIN);
    DDRB |= (1 << IN2_PIN);

    // 2. Setup Timer0 for Fast PWM on the ENA pin (PD6/OC0A)
    // Use non-inverting mode for OC0A, and Fast PWM mode (counts 0-255).
    TCCR0A = (1 << COM0A1) | (1 << WGM01) | (1 << WGM00);
    
    // 3. Start the timer with a prescaler of 64.
    // This gives a PWM frequency of ~977 Hz, which is good for DC motors.
    TCCR0B = (1 << CS01) | (1 << CS00);
    
    // Ensure motor is stopped initially
    motor_stop();
}

void motor_set_speed(uint8_t speed) {
    // Write the speed value directly to the Output Compare Register for Timer0 A.
    // This controls the PWM duty cycle.
    OCR0A = speed;
}

void motor_forward(void) {
    // Set IN1 HIGH and IN2 LOW for forward direction
    PORTD |= (1 << IN1_PIN);
    PORTB &= ~(1 << IN2_PIN);
}

void motor_backward(void) {
    // Set IN1 LOW and IN2 HIGH for backward direction
    PORTD &= ~(1 << IN1_PIN);
    PORTB |= (1 << IN2_PIN);
}

void motor_stop(void) {
    // Set both IN1 and IN2 LOW to let the motor coast to a stop.
    PORTD &= ~(1 << IN1_PIN);
    PORTB &= ~(1 << IN2_PIN);
}