#ifndef GYROSCOPE_H
#define GYROSCOPE_H

//Dependencies
#include <Wire.h>
#include <Arduino_LSM6DSOX.h>
#include <micro_ros_arduino.h>
#include <sensor_msgs/msg/imu.h>


//I2C Address of the gyro on Edurob Board
#define GYRO_IMU_I2C_ADDR 0x6B
#define SAMPLING_AMOUNT 100

//Conversion constants
#define CONVERS_CONST_DEGREES_PER_SECOND_2_RADIANS_PER_SECOND 0.017453
#define CONVERS_CONST_G_2_A 9.8066

void init_IMU();
bool fetch_gyro_data(float &x, float &y, float &z);
bool fetch_acceleration_data(float &x, float &y, float &z);
void read_imu_data(sensor_msgs__msg__Imu &gyro_imu_msg);
void quartenion_algorithm(float &angle_x, float &angle_y, float &angle_z);

#endif // GYROSCOPE_H