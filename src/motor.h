#ifndef MOTOR_H
#define MOTOR_H

#include <stdint.h> // Required for the uint8_t type

// Initializes the hardware pins and Timer0 for PWM speed control.
// This must be called once in your main setup.
void motor_init(void);

// Sets the motor speed.
// speed: A value from 0 (stopped) to 255 (full speed).
void motor_set_speed(uint8_t speed);

// Commands the motor to run forward.
void motor_forward(void);

// Commands the motor to run backward.
void motor_backward(void);

// Commands the motor to stop (coast).
void motor_stop(void);

#endif // MOTOR_H