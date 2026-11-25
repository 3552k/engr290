// imu.h - MPU-6050 driver for the hovercraft (SDA = PC4, SCL = PC5)

#ifndef IMU_H
#define IMU_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

uint8_t IMU_Init(void);              // 1 = IMU found
uint8_t IMU_Calibrate(void);         // ~2 s, still, fans off. 1 = OK
void    IMU_Update(void);            // call every loop
float   IMU_Yaw(void);               // degrees, -180..+180
void    IMU_SetTarget(float change); // target = yaw + change
uint8_t IMU_TurnFinished(void);      // 1 = within tolerance of target
float   IMU_Steer(void);             // servo correction in degrees

float   IMU_Target(void);
float   IMU_HeadingError(void);      // target - yaw, -180..+180
float   IMU_YawRate(void);           // deg/s
uint8_t IMU_Connected(void);

#ifdef __cplusplus
}
#endif

#endif
