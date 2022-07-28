#ifndef _GRIPPER_H
#define _GRIPPER_H

#include "BMI088.h"
#include "VL53L0x.h"
#include "bsp_def.h"
#include "bsp_log.h"
#include "buzzer.h"
#include "can_motor.h"
#include "pub_sub.h"
#include "stdint.h"
#include "super_cap_wuli.h"

#pragma pack(1)

typedef struct Gripper_t {
    BMI088_imu *imu;
    buzzer *internal_buzzer;
} Gripper;
#pragma pack()

Gripper *Gripper_Create(void);
void Gripper_Update(Gripper *obj);
#endif