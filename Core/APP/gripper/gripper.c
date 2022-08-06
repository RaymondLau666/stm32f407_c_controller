#include "gripper.h"

#include <arm_math.h>
#include <math.h>

#include "app.h"
#include "bsp.h"
#include "common.h"

#ifdef MAIN_BOARD
int motor1_spd = 0;
int motor2_spd = 0;
void Motor1_Setspd(int spd, Gripper *obj);
void Motor2_Setspd(int spd, Gripper *obj);
#else

#endif

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
#ifdef MAIN_BOARD
    uint32_t music_id = 8;
#else
    uint32_t music_id = 2;
#endif
    internal_buzzer_config.music = musics[music_id];
    internal_buzzer_config.len = music_lens[music_id];
    internal_buzzer_config.bsp_pwm_index = PWM_BUZZER_PORT;
    obj->internal_buzzer = Buzzer_Create(&internal_buzzer_config);

#ifdef MAIN_BOARD

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
    send_config.can_identifier = 0x003;
    recv_config.can_identifier = 0x004;
    send_config.data_len = sizeof(Main_board_send_data);
    recv_config.data_len = sizeof(Secondary_board_send_data);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = NULL;
    obj->send = CanSend_Create(&send_config);
    obj->recv = CanRecv_Create(&recv_config);
    obj->recv_data = (Secondary_board_send_data *)obj->recv->data_rx.data;

    //遥控器
    dt7_config remote_config;
    remote_config.bsp_uart_index = UART_REMOTE_PORT;
    remote_config.lost_callback = NULL;
    obj->remote = dt7_Create(&remote_config);

    // 舵机1
    Servo_config servo_1_config;
    servo_1_config.model = MODEL_POS;
    servo_1_config.bsp_pwm_index = PWM_SERVO_1_PORT;
    servo_1_config.max_angle = 180;
    servo_1_config.initial_angle = 90;
    obj->servo_1 = Servo_Create(&servo_1_config);

    // 舵机2
    Servo_config servo_2_config;
    servo_2_config.model = MODEL_POS;
    servo_2_config.bsp_pwm_index = PWM_SERVO_2_PORT;
    servo_2_config.max_angle = 180;
    servo_2_config.initial_angle = 90;
    obj->servo_2 = Servo_Create(&servo_2_config);

    // 舵机3
    Servo_config servo_3_config;
    servo_3_config.model = MODEL_POS;
    servo_3_config.bsp_pwm_index = PWM_SERVO_3_PORT;
    servo_3_config.max_angle = 180;
    servo_3_config.initial_angle = 90;
    obj->servo_3 = Servo_Create(&servo_3_config);

    // 舵机4
    Servo_config servo_4_config;
    servo_4_config.model = MODEL_POS;
    servo_4_config.bsp_pwm_index = PWM_SERVO_4_PORT;
    servo_4_config.max_angle = 180;
    servo_4_config.initial_angle = 90;
    obj->servo_4 = Servo_Create(&servo_4_config);

    // 舵机5
    Servo_config servo_5_config;
    servo_5_config.model = MODEL_POS;
    servo_5_config.bsp_pwm_index = PWM_SERVO_5_PORT;
    servo_5_config.max_angle = 180;
    servo_5_config.initial_angle = 90;
    obj->servo_5 = Servo_Create(&servo_5_config);

    // 舵机6
    Servo_config servo_6_config;
    servo_6_config.model = MODEL_POS;
    servo_6_config.bsp_pwm_index = PWM_SERVO_6_PORT;
    servo_6_config.max_angle = 180;
    servo_6_config.initial_angle = 90;
    obj->servo_6 = Servo_Create(&servo_6_config);

    // 舵机7
    Servo_config servo_7_config;
    servo_7_config.model = MODEL_POS;
    servo_7_config.bsp_pwm_index = PWM_SERVO_7_PORT;
    servo_7_config.max_angle = 180;
    servo_7_config.initial_angle = 90;
    obj->servo_7 = Servo_Create(&servo_7_config);
#else

    // 板间通信配置
    can_send_config send_config;
    can_recv_config recv_config;
    send_config.bsp_can_index = 1;
    recv_config.bsp_can_index = 1;
    send_config.can_identifier = 0x004;
    recv_config.can_identifier = 0x003;
    send_config.data_len = sizeof(Secondary_board_send_data);
    recv_config.data_len = sizeof(Main_board_send_data);
    recv_config.notify_func = NULL;
    recv_config.lost_callback = NULL;
    obj->send = CanSend_Create(&send_config);
    obj->recv = CanRecv_Create(&recv_config);
    obj->recv_data = (Main_board_send_data *)obj->recv->data_rx.data;

    // 舵机1
    Servo_config servo_1_config;
    servo_1_config.model = MODEL_POS;
    servo_1_config.bsp_pwm_index = PWM_SERVO_1_PORT;
    servo_1_config.max_angle = 180;
    servo_1_config.initial_angle = 90;
    obj->servo_1 = Servo_Create(&servo_1_config);

    // 舵机2
    Servo_config servo_2_config;
    servo_2_config.model = MODEL_POS;
    servo_2_config.bsp_pwm_index = PWM_SERVO_2_PORT;
    servo_2_config.max_angle = 180;
    servo_2_config.initial_angle = 90;
    obj->servo_2 = Servo_Create(&servo_2_config);

    // 舵机3
    Servo_config servo_3_config;
    servo_3_config.model = MODEL_POS;
    servo_3_config.bsp_pwm_index = PWM_SERVO_3_PORT;
    servo_3_config.max_angle = 180;
    servo_3_config.initial_angle = 90;
    obj->servo_3 = Servo_Create(&servo_3_config);

    // 舵机4
    Servo_config servo_4_config;
    servo_4_config.model = MODEL_POS;
    servo_4_config.bsp_pwm_index = PWM_SERVO_4_PORT;
    servo_4_config.max_angle = 180;
    servo_4_config.initial_angle = 90;
    obj->servo_4 = Servo_Create(&servo_4_config);

    // 舵机5
    Servo_config servo_5_config;
    servo_5_config.model = MODEL_POS;
    servo_5_config.bsp_pwm_index = PWM_SERVO_5_PORT;
    servo_5_config.max_angle = 180;
    servo_5_config.initial_angle = 90;
    obj->servo_5 = Servo_Create(&servo_5_config);

    // 舵机6
    Servo_config servo_6_config;
    servo_6_config.model = MODEL_POS;
    servo_6_config.bsp_pwm_index = PWM_SERVO_6_PORT;
    servo_6_config.max_angle = 180;
    servo_6_config.initial_angle = 90;
    obj->servo_6 = Servo_Create(&servo_6_config);

    // 舵机7
    Servo_config servo_7_config;
    servo_7_config.model = MODEL_POS;
    servo_7_config.bsp_pwm_index = PWM_SERVO_7_PORT;
    servo_7_config.max_angle = 180;
    servo_7_config.initial_angle = 90;
    obj->servo_7 = Servo_Create(&servo_7_config);

    // // 电机 1+
    // Servo_config motor_1_positive_config;
    // motor_1_positive_config.model = MODEL_POS;
    // motor_1_positive_config.bsp_pwm_index = PWM_SERVO_1_PORT;
    // motor_1_positive_config.max_angle = 180;
    // motor_1_positive_config.initial_angle = 0;
    // obj->motor_1_positive = Servo_Create(&motor_1_positive_config);

    // // 电机 1-
    // Servo_config motor_1_nagitive_config;
    // motor_1_nagitive_config.model = MODEL_POS;
    // motor_1_nagitive_config.bsp_pwm_index = PWM_SERVO_2_PORT;
    // motor_1_nagitive_config.max_angle = 180;
    // motor_1_nagitive_config.initial_angle = 0;
    // obj->motor_1_nagitive = Servo_Create(&motor_1_nagitive_config);

    // // 电机 2+
    // Servo_config motor_2_positive_config;
    // motor_2_positive_config.model = MODEL_POS;
    // motor_2_positive_config.bsp_pwm_index = PWM_SERVO_3_PORT;
    // motor_2_positive_config.max_angle = 180;
    // motor_2_positive_config.initial_angle = 0;
    // obj->motor_2_positive = Servo_Create(&motor_2_positive_config);

    // // 电机 2-
    // Servo_config motor_2_nagitive_config;
    // motor_2_nagitive_config.model = MODEL_POS;
    // motor_2_nagitive_config.bsp_pwm_index = PWM_SERVO_4_PORT;
    // motor_2_nagitive_config.max_angle = 180;
    // motor_2_nagitive_config.initial_angle = 0;
    // obj->motor_2_nagitive = Servo_Create(&motor_2_nagitive_config);

#endif

    return obj;
}

#ifdef MAIN_BOARD
void Gripper_Update(Gripper *obj) {
    if (obj->imu->bias_init_success) {
        if (!buzzer_started) {
            buzzer_started = 1;
            Buzzer_Start(obj->internal_buzzer);
        }
    }

    // static double delta_i = 0;
    // static int sig = 1;
    // obj->servo_2->pos_servo_control = 90 + delta_i;
    // if (delta_i > 90 || delta_i < -90) sig *= -1;
    // delta_i += sig * 0.1;
    // obj->servo_1->pos_servo_control = 180;

    obj->send_data.motor1_spd = obj->remote->data.rc.ch3 - 1024;
    obj->send_data.motor2_spd = obj->remote->data.rc.ch1 - 1024;
    CanSend_Send(obj->send, (uint8_t *)&(obj->send_data));  // 板间通信

    // Motor1_Setspd(motor1_spd, obj);
    // Motor2_Setspd(motor2_spd, obj);
}
// void Motor1_Setspd(int spd, Gripper *obj) {
//     if (spd == 0) {
//         obj->motor_1_positive->pos_servo_control = 0;
//         obj->motor_1_nagitive->pos_servo_control = 0;
//     } else if (spd > 0) {
//         obj->motor_1_positive->pos_servo_control = spd;
//         obj->motor_1_nagitive->pos_servo_control = 0;
//     } else if (spd < 0) {
//         obj->motor_1_positive->pos_servo_control = 0;
//         obj->motor_1_nagitive->pos_servo_control = spd;
//     }
// }
// void Motor2_Setspd(int spd, Gripper *obj) {
//     if (spd == 0) {
//         obj->motor_2_positive->pos_servo_control = 0;
//         obj->motor_2_nagitive->pos_servo_control = 0;
//     } else if (spd > 0) {
//         obj->motor_2_positive->pos_servo_control = spd;
//         obj->motor_2_nagitive->pos_servo_control = 0;
//     } else if (spd < 0) {
//         obj->motor_2_positive->pos_servo_control = 0;
//         obj->motor_2_nagitive->pos_servo_control = spd;
//     }
// }
#else

void Gripper_Update(Gripper *obj) {
    if (obj->imu->bias_init_success) {
        if (!buzzer_started) {
            buzzer_started = 1;
            Buzzer_Start(obj->internal_buzzer);
        }
    }

    // motor1_spd = obj->recv_data->motor1_spd;
    // motor2_spd = obj->recv_data->motor2_spd;
}

#endif
