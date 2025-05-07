#pragma once

#include "MPU6050.h"
#include <bmp280.h>

#define PIN_SDA 21
#define PIN_CLK 22
#define I2C_PORT I2C_NUM_0

struct mpu6050_handler_t {
private:
    uint16_t packetSize = 42;    // expected DMP packet size (default is 42 bytes)
    uint16_t fifoCount;          // count of all bytes currently in FIFO
    uint8_t fifoBuffer[64];      // FIFO storage buffer
    uint8_t mpuIntStatus;        // holds actual interrupt status byte from MPU
    MPU6050 mpu;                 // mpu6050 instance
    
public:
    Quaternion q;                // [w, x, y, z]         quaternion container
    VectorFloat gravity;         // [x, y, z]            gravity vector
    float ypr[3];                // [yaw, pitch, roll]   yaw/pitch/roll container

    esp_err_t initialize();
    void read_data();
    void print_data() const;
};

class bmp280_handler{
    bmp280_t bmp_dev;

public:
    float pressure;
    float temperature; 
    float humidity;

    esp_err_t initialize();
    void read_data();
    void print_data() const;
};

esp_err_t init_sensors(mpu6050_handler* mpu_handler, bmp280_handler* bmp_handler);
void read_sensors(mpu6050_handler* mpu_handler, bmp280_handler* bmp_handler);