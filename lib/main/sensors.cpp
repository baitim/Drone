#include <stdio.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <esp_system.h>
#include <bmp280.h>
#include <driver/i2c.h>
#include <esp_log.h>
#include <esp_err.h>
#include "MPU6050.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "i2cdev.h"
#include <iostream>


#include "sensors.hpp"

void bmp280_handler_t::print_data() const {
    std::cout << "pressure: "    << pressure    << ", "
              << "temperature: " << temperature << "\n";
}

void bmp280_handler_t::read_data() {
    if (bmp280_read_float(&bmp_dev, &temperature, &pressure, &humidity) == ESP_OK)
        return;

    std::cout << "\n\nBMP280: Failed to read data\n\n";
}

esp_err_t bmp280_handler_t::initialize() {
    esp_err_t result = ESP_OK;
    bmp280_params_t params;
    bmp280_init_default_params(&params);
    memset(&bmp_dev, 0, sizeof(bmp280_t));

    result = bmp280_init_desc(&bmp_dev, BMP280_I2C_ADDRESS_0, I2C_PORT, GPIO_NUM_21, GPIO_NUM_22);
    if (result != ESP_OK) {
        std::cout << "Failed to initialize BMP280: " << result << "\n";
        vTaskDelete(NULL);
        return result;
    }

    result = bmp280_init(&bmp_dev, &params);
    if (result != ESP_OK) {
        std::cout << "Failed to configure BMP280: " << result << "\n";
        vTaskDelete(NULL);
        return result;
    }

    bool bme280p = (bmp_dev.id == BME280_CHIP_ID);
    std::cout << "BMP280 found: "<< (bme280p ? "BME280" : "BMP280") << "\n";

    return ESP_OK;
}

void mpu6050_handler_t::read_data() {
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();

    if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
        mpu.resetFIFO();
    } else if (mpuIntStatus & 0x02) {
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
        mpu.getFIFOBytes(fifoBuffer, packetSize);
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        
        mpu.dmpGetGravity(&gravity, &q);
        mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    }
}

void mpu6050_handler_t::print_data() const {
    std::cout << "mpu6050 data:\n";
    std::cout << "\tYAW:   " << ypr[0] * 180 / M_PI
              << "\tPITCH: " << ypr[1] * 180 / M_PI
              << "\tROLL:  " << ypr[2] * 180 / M_PI
              << "g: [" << gravity.x <<  "," << gravity.y << ", " << gravity.z << "]";
}

esp_err_t mpu6050_handler_t::initialize() {
    mpu.initialize();
    mpu.dmpInitialize();
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.setDMPEnabled(true);
    std::cout << "MPU6050 initialized and calibrated.\n";
    return ESP_OK;
}


esp_err_t init_sensors(mpu6050_handler_t& mpu_handler, bmp280_handler_t& bmp_handler) {
    esp_err_t result = i2cdev_init();
    if (result != ESP_OK) {
        std::cout << "Failed to initialize i2cdev: " << result << "\n";
        vTaskDelete(NULL);
        return result;
    }

    result = bmp_handler.initialize();

    if (result != ESP_OK) {
        std::cout << "Failed to initialize bmp280: " << result << "\n";
        vTaskDelete(NULL);
        return result;
    }

    mpu_handler.initialize();
    return ESP_OK;
}

void read_sensors(mpu6050_handler_t& mpu_handler, bmp280_handler_t& bmp_handler) {
    mpu_handler.read_data();
    bmp_handler.read_data();
}