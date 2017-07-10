#ifndef __GYRO_H__
#define __GYRO_H__

#include "debug.h"

int16_t gyro_get_x_acc(void);
int16_t gyro_get_y_acc(void);
int16_t gyro_get_z_acc(void);
uint8_t gyro_get_calibration(void);
int16_t gyro_get_angle(void);
void gyro_set_angle(int16_t angle);

void set_gyro_on(void);
bool wait_for_gyro_on(void);
bool check_gyro_stable(void);
void set_gyro_off(void);
void set_gyro_status(void);
void reset_gyro_status(void);
uint8_t is_gyro_on(void);
#if GYRO_DYNAMIC_ADJUSTMENT
void set_gyro_dynamic_on(void);
void set_gyro_dynamic_off(void);
#endif
#endif /* __GYRO_H */
