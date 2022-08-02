#include "gripper.h"

#include <arm_math.h>
#include <math.h>

#include "bsp.h"
#include "common.h"

uint8_t buzzer_started = 0;
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

    // 舵机
    Servo_config servo_1_config;
    servo_1_config.model = MODEL_POS;
    servo_1_config.bsp_pwm_index = PWM_SERVO_1_PORT;
    servo_1_config.max_angle = 180;
    servo_1_config.initial_angle = 90;
    obj->servo_1 = Servo_Create(&servo_1_config);

    // 舵机
    Servo_config servo_2_config;
    servo_2_config.model = MODEL_POS;
    servo_2_config.bsp_pwm_index = PWM_SERVO_2_PORT;
    servo_2_config.max_angle = 180;
    servo_2_config.initial_angle = 90;
    obj->servo_2 = Servo_Create(&servo_2_config);

    // 舵机
    Servo_config servo_3_config;
    servo_3_config.model = MODEL_POS;
    servo_3_config.bsp_pwm_index = PWM_SERVO_3_PORT;
    servo_3_config.max_angle = 180;
    servo_3_config.initial_angle = 90;
    obj->servo_3 = Servo_Create(&servo_3_config);

    // 舵机
    Servo_config servo_4_config;
    servo_4_config.model = MODEL_POS;
    servo_4_config.bsp_pwm_index = PWM_SERVO_4_PORT;
    servo_4_config.max_angle = 180;
    servo_4_config.initial_angle = 90;
    obj->servo_4 = Servo_Create(&servo_4_config);

    // 舵机
    Servo_config servo_5_config;
    servo_5_config.model = MODEL_POS;
    servo_5_config.bsp_pwm_index = PWM_SERVO_5_PORT;
    servo_5_config.max_angle = 180;
    servo_5_config.initial_angle = 90;
    obj->servo_5 = Servo_Create(&servo_5_config);

    // 舵机
    Servo_config servo_6_config;
    servo_6_config.model = MODEL_POS;
    servo_6_config.bsp_pwm_index = PWM_SERVO_6_PORT;
    servo_6_config.max_angle = 180;
    servo_6_config.initial_angle = 90;
    obj->servo_6 = Servo_Create(&servo_6_config);

    // 舵机
    Servo_config servo_7_config;
    servo_7_config.model = MODEL_POS;
    servo_7_config.bsp_pwm_index = PWM_SERVO_7_PORT;
    servo_7_config.max_angle = 180;
    servo_7_config.initial_angle = 90;
    obj->servo_7 = Servo_Create(&servo_7_config);

    return obj;
}

void Gripper_Update(Gripper *obj) {
    static double delta_i = 0;
    static int sig = 1;
    obj->servo_2->pos_servo_control = 90 + delta_i;
    if (delta_i > 90 || delta_i < -90) sig *= -1;
    delta_i += sig * 0.1;

    obj->servo_1->pos_servo_control = 180;

    if (obj->imu->bias_init_success) {
        if (!buzzer_started) {
            buzzer_started = 1;
            // Buzzer_Start(obj->internal_buzzer);
        }
    }
}
