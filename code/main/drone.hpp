#pragma once

#include <algorithm>
#include <iostream>
#include <sstream>
#include <vector>
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "sensors.hpp"
#include "motors.hpp"

constexpr int MOTOR_COUNT = 4;

class drone_t {
    std::vector<motor_t> motors;
    mpu6050_handler_t mpu6050;
    bmp280_handler_t  bmp280;

    double YPR_tar[3] = {};

    double YPR_diff_n [3] = {}; //YPR diff on this iteration
    double YPR_diff_n1[3] = {}; //YPR diff on previous iteration
    double YPR_diff_n2[3] = {}; //YPR diff on prev-previous iteration
    
    //Y  P  R
    double PID_Kprop[3] = {};
    double PID_Kdiff[3] = {};
    double PID_Kintg[3] = {};

    double Control_val_YPR[3] = {};

    double min_throttle = 2;
    double max_throttle = 30;
    double throttle = 0;

public:
    void initialize_motors_and_timer(
        ledc_timer_bit_t timer_resolution, ledc_mode_t timer_mode, uint32_t frequency, int min_period_ms, int max_period_ms,
        int GPIO_UP_L, int GPIO_UP_R, int GPIO_DOWN_R, int GPIO_DOWN_L) {

        ledc_timer_config_t ledc_timer = {
            .speed_mode       = timer_mode,
            .duty_resolution  = timer_resolution,
            .timer_num        = LEDC_TIMER_0,
            .freq_hz          = frequency,
            .clk_cfg          = LEDC_AUTO_CLK,
            .deconfigure      = false
        };
        ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

        motors = {
            {LEDC_CHANNEL_0, GPIO_UP_L,   timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms},
            {LEDC_CHANNEL_1, GPIO_UP_R,   timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms},
            {LEDC_CHANNEL_2, GPIO_DOWN_R, timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms},
            {LEDC_CHANNEL_3, GPIO_DOWN_L, timer_resolution, timer_mode, frequency, min_period_ms, max_period_ms}
        };

        for (int i = 0; i < MOTOR_COUNT; ++i) {
            motors[i].init_motor();
        }
    };

    void initialize_sensors() {
        init_sensors(mpu6050, bmp280);
    };

    void set_duty(short index, double percents) {
        motors[index].set_throttle(percents);
        motors[index].update_duty();
    };

    void set_duty(double percents) {
        for (int i = 0; i < MOTOR_COUNT; ++i) {
            if (motors[i].get_throttle() != percents) {
                motors[i].set_throttle(percents);
                motors[i].update_duty();
            }
        }
    }

    void read_sensors_values() {
        read_sensors(mpu6050, bmp280);
    }

    void print_state() {
        std::cout << "\033[0H\033[0J";

        mpu6050.print_data();
        bmp280.print_data();

        std::ostringstream oss;
        oss << "T_YAW  : " << YPR_tar[0]
            << "T_PITCH: " << YPR_tar[1]
            << "T_ROLL : " << YPR_tar[2]
            << "Vals: " << Control_val_YPR[0] << ", " <<  Control_val_YPR[1] << ", " <<  Control_val_YPR[2]
            << "Prop: " << PID_Kprop[0] << ", " << PID_Kprop[1] << ", " << PID_Kprop[2] << '\n'
            << "Intg: " << PID_Kintg[0] << ", " << PID_Kintg[1] << ", " << PID_Kintg[2] << '\n'
            << "Diff: " << PID_Kdiff[0] << ", " << PID_Kdiff[1] << ", " << PID_Kdiff[2] << '\n';
        std::cout << oss.str();
    }

    void set_targets(double Y, double P, double R) {
        YPR_tar[0] = Y;
        YPR_tar[1] = P;
        YPR_tar[2] = R;
    }

    void set_throttle(double tar_throttle) {
        throttle = std::clamp(tar_throttle, min_throttle, max_throttle);
    }

    void force_set_throttle(double tar_throttle) {
        throttle = std::clamp(tar_throttle, 0.0, max_throttle);
    }

    void set_PID(double PID_Kprop_new[3], double PID_Kintg_new[3], double PID_Kdiff_new[3]) {
        for (int i = 0; i < 3; ++i) {
            PID_Kprop[i] = PID_Kprop_new[i];
            PID_Kdiff[i] = PID_Kdiff_new[i];
            PID_Kintg[i] = PID_Kintg_new[i];
        }
    }

    void update_motors() {
        /*

    -Y<-  +P  ->+Y
        0   1
         \ /
    +R    ^    -R
         / \
        3   2
          -P
        
        */

        auto& val0 = Control_val_YPR[0];
        auto& val1 = Control_val_YPR[1];
        auto& val2 = Control_val_YPR[2];

        set_duty(0, std::clamp(throttle + val0 - val1 - val2, min_throttle, max_throttle));
        set_duty(1, std::clamp(throttle - val0 - val1 + val2, min_throttle, max_throttle));
        set_duty(2, std::clamp(throttle + val0 + val1 + val2, min_throttle, max_throttle));
        set_duty(3, std::clamp(throttle - val0 + val1 - val2, min_throttle, max_throttle));
    }

    void processPID() {
        //replacing iteration steps with new ones
        //getting new controlling values on Yaw (0), Pitch (1) and Roll (2)

        bool YPR_tune[3] = {true, true, true};

        double val = 0;
        for (int i = 0; i < 3; i++) {
            if (YPR_tune[i]) {
                YPR_diff_n2[i] = YPR_diff_n1[i];
                YPR_diff_n1[i] = YPR_diff_n[i];
                YPR_diff_n[i]  = YPR_tar[i] - mpu6050.ypr[i];
    
                /* U(n) =   U(n-1) + 
                            Kp * (E(n) - E(n-1)) +
                            Ki * E(n) + 
                            Kd * (E(n) - 2E(n-1) + E(n-2))
                */

                val =   Control_val_YPR[i] + 
                        PID_Kprop[i] * (YPR_diff_n[i] - YPR_diff_n1[i]) + 
                        PID_Kintg[i] *  YPR_diff_n[i] + 
                        PID_Kdiff[i] * (YPR_diff_n[i] - 2 * YPR_diff_n1[i] + YPR_diff_n2[i]);
                Control_val_YPR[i] = val;
            }
        }
    }
};