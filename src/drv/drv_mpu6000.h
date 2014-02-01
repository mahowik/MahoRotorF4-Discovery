//drv_mpu6000.h
#pragma once

bool mpu6000Detect(sensor_t * acc, sensor_t * gyro, uint16_t lpf, uint8_t *scale);
void mpu6000DmpLoop(void);
void mpu6000DmpResetFifo(void);
