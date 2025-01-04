#ifndef _IMU_H
#define _IMU_H


#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <ESP32Servo.h>

extern Adafruit_MPU6050 mpu;
extern Servo myservo;
extern double x_accel_calib_rate;
extern double y_accel_calib_rate;
extern double yaw_vel_calib_rate;

void Init_MPU6050_func();
void IMU_Get_Data_func();
void IMU_Calib_func();


#endif