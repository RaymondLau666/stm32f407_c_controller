#include "gripper.h"

#include <arm_math.h>
#include <math.h>

#include "bsp.h"
#include "common.h"

uint8_t buzzer_started=0;
Gripper *Gripper_Create() {
    Gripper *obj = (Gripper *)malloc(sizeof(Gripper));
    memset(obj, 0, sizeof(Gripper));

    // 外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 25.0f;  //设定温度为x度
    internal_imu_config.lost_callback = NULL;
    internal_imu_config.imu_axis_convert[0] = 1;
    internal_imu_config.imu_axis_convert[1] = 2;
    internal_imu_config.imu_axis_convert[2] = 3;
    obj->imu = BMI088_Create(&internal_imu_config);

    //蜂鸣器配置
    buzzer_config internal_buzzer_config;
    uint32_t music_id = 8;
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    obj->internal_buzzer = Buzzer_Create(&internal_buzzer_config);

    return obj;
}

void Gripper_Update(Gripper *obj) {
    if (obj->imu->bias_init_success) {
        if(!buzzer_started){buzzer_started = 1;
        Buzzer_Start(obj->internal_buzzer);}
    }
    
}
