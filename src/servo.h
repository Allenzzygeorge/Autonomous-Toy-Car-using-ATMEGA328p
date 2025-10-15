#ifndef SERVOS_H
#define SERVOS_H

// Initializes Timer1 for BOTH the steering and rotation servos.
// This is the only init function you need to call.
void servos_init(void);

// --- Steering Servo Functions ---
void steer_center(void);
void steer_right(void);
void steer_left(void);

// --- Rotation Servo Functions ---
void uss_rot_initial(void);
void uss_rot_first(void);
void uss_rot_second(void);

#endif // SERVOS_H