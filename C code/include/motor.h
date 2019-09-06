/**
 * @file
 *
 * DoroboLib32 Motor (DM)
 *
 * Functions to set motor speed.
 *
 * Copyright (c) 2016 Laurent Schröder, Claus Fühner, Michael Hoffmann
 */

#ifndef DOROBO32_INC_MOTOR_H_
#define DOROBO32_INC_MOTOR_H_

#include <stdint.h>

/**
 * Dorobo32 motors
 */
enum DM_MOTORS_E
{
    DM_MOTOR0 = 0, ///< Motor 0
    DM_MOTOR1 = 1, ///< Motor 1
    DM_MOTOR2 = 2, ///< Motor 2
    DM_MOTOR3 = 3  ///< Motor 3
};

#ifdef __cplusplus
 extern "C" {
#endif

/**
 * @brief   Motor initialization.
 *
 */
void motor_init(void);

/**
 * @brief Get the current speed of the given motor.
 *
 * @param motor Motor to control
 * @return speed Current motor speed in percent (-100-100)
 */
int8_t motor_get_speed(enum DM_MOTORS_E motor);

/**
 * @brief   Set the speed for the given motor.
 *
 * Warning: The user of this function is responsible for
 * limiting the motor current by controlling the slope of
 * speed changes.
 *
 * @param   motor Motor to control
 * @param   speed New motor speed and direction in percent (-100-100)
 */
void motor_set(enum DM_MOTORS_E motor, int8_t speed);

#ifdef __cplusplus
}
#endif

#endif /* DOROBO32_INC_MOTOR_H_ */
