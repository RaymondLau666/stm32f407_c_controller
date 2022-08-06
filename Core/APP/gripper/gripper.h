#ifndef _GRIPPER_H
#define _GRIPPER_H

#include "BMI088.h"
#include "DT7_DR16.h"
#include "VL53L0x.h"
#include "app.h"
#include "bsp_def.h"
#include "bsp_log.h"
#include "buzzer.h"
#include "can_motor.h"
#include "can_recv.h"
#include "can_send.h"
#include "pub_sub.h"
#include "pwm_servo.h"
#include "stdint.h"

#pragma pack(1)
typedef struct Gimbal_board_send_t {
    uint8_t motor1_spd;
    uint8_t motor2_spd;
} Main_board_send_data;

// 云台<-底盘数据包
typedef struct Chassis_board_send_t {
    uint8_t useless;
} Secondary_board_send_data;

typedef struct Gripper_t {
#ifdef MAIN_BOARD
    BMI088_imu *imu;
    buzzer *internal_buzzer;
    dt7Remote *remote;

    can_recv *recv;
    can_send *send;
    Main_board_send_data send_data;
    Secondary_board_send_data *recv_data;

    Servo *servo_1;
    Servo *servo_2;
    Servo *servo_3;
    Servo *servo_4;
    Servo *servo_5;
    Servo *servo_6;
    Servo *servo_7;
#else
    BMI088_imu *imu;
    buzzer *internal_buzzer;

    can_recv *recv;
    can_send *send;
    Secondary_board_send_data send_data;
    Main_board_send_data *recv_data;

    // Servo *motor_1_positive;
    // Servo *motor_1_nagitive;
    // Servo *motor_2_positive;
    // Servo *motor_2_nagitive;

    Servo *servo_1;
    Servo *servo_2;
    Servo *servo_3;
    Servo *servo_4;
    Servo *servo_5;
    Servo *servo_6;
    Servo *servo_7;
#endif
} Gripper;

#pragma pack()

Gripper *Gripper_Create(void);
void Gripper_Update(Gripper *obj);
#endif