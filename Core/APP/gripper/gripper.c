#include "gripper.h"

#include <arm_math.h>
#include <math.h>

#include "app.h"
#include "bsp.h"
#include "common.h"

uint8_t buzzer_started = 0;

#ifdef MAIN_BOARD
float servo_1_target_degree = 140;
int motor1_spd = 0;
int motor2_spd = 0;
void Mode_Update(Gripper *obj);
void Up_and_Down_z_Update(Gripper *obj);
void Fowardback_and_Roll_xy_Update(Gripper *obj);
void Clamp_Update(Gripper *obj);  //夹爪抓取放下
void Side_Updown(Gripper *obj);   //侧面机构(带着电机的)抬升和下降

void Motor1_Setspd(int spd, Gripper *obj);
void Motor2_Setspd(int spd, Gripper *obj);
#else

#endif

Gripper *Gripper_Create() {
    Gripper *obj = (Gripper *)malloc(sizeof(Gripper));
    memset(obj, 0, sizeof(Gripper));

    // 外设初始化
    BMI088_config internal_imu_config;
    internal_imu_config.bsp_gpio_accel_index = GPIO_BMI088_ACCEL_NS;
    internal_imu_config.bsp_gpio_gyro_index = GPIO_BMI088_GYRO_NS;
    internal_imu_config.bsp_pwm_heat_index = PWM_BMI088_HEAT_PORT;
    internal_imu_config.bsp_spi_index = SPI_BMI088_PORT;
    internal_imu_config.temp_target = 45.0f;  //设定温度为x度
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

    // 舵机1  气室
    Servo_config servo_1_config;
    servo_1_config.model = MODEL_POS;
    servo_1_config.bsp_pwm_index = PWM_SERVO_1_PORT;
    servo_1_config.max_angle = 180;
    servo_1_config.initial_angle = 140;
    obj->servo_1 = Servo_Create(&servo_1_config);
    // 舵机2  夹爪1
    Servo_config servo_2_config;
    servo_2_config.model = MODEL_POS;
    servo_2_config.bsp_pwm_index = PWM_SERVO_2_PORT;
    servo_2_config.max_angle = 180;
    servo_2_config.initial_angle = 5;
    obj->servo_2 = Servo_Create(&servo_2_config);
    // 舵机3  夹爪2
    Servo_config servo_3_config;
    servo_3_config.model = MODEL_POS;
    servo_3_config.bsp_pwm_index = PWM_SERVO_3_PORT;
    servo_3_config.max_angle = 180;
    servo_3_config.initial_angle = 180;
    obj->servo_3 = Servo_Create(&servo_3_config);
    // 电机 1+
    Servo_config motor_1_positive_config;
    motor_1_positive_config.model = MODEL_POS;
    motor_1_positive_config.bsp_pwm_index = PWM_SERVO_4_PORT;
    motor_1_positive_config.max_angle = 180;
    motor_1_positive_config.initial_angle = 0;
    obj->motor_1_positive = Servo_Create(&motor_1_positive_config);
    // 电机 1-
    Servo_config motor_1_nagitive_config;
    motor_1_nagitive_config.model = MODEL_POS;
    motor_1_nagitive_config.bsp_pwm_index = PWM_SERVO_5_PORT;
    motor_1_nagitive_config.max_angle = 180;
    motor_1_nagitive_config.initial_angle = 0;
    obj->motor_1_nagitive = Servo_Create(&motor_1_nagitive_config);
    // 电机 2+
    Servo_config motor_2_positive_config;
    motor_2_positive_config.model = MODEL_POS;
    motor_2_positive_config.bsp_pwm_index = PWM_SERVO_6_PORT;
    motor_2_positive_config.max_angle = 180;
    motor_2_positive_config.initial_angle = 0;
    obj->motor_2_positive = Servo_Create(&motor_2_positive_config);
    // 电机 2-
    Servo_config motor_2_nagitive_config;
    motor_2_nagitive_config.model = MODEL_POS;
    motor_2_nagitive_config.bsp_pwm_index = PWM_SERVO_7_PORT;
    motor_2_nagitive_config.max_angle = 180;
    motor_2_nagitive_config.initial_angle = 0;
    obj->motor_2_nagitive = Servo_Create(&motor_2_nagitive_config);

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
    Mode_Update(obj);
    Up_and_Down_z_Update(obj);                              //整体z方向运动
    Fowardback_and_Roll_xy_Update(obj);                     //整体水平面方向运动
    Clamp_Update(obj);                                      //夹爪抓取放下
    Side_Updown(obj);                                       //从板：侧面机构(带着电机的)抬升和下降
    CanSend_Send(obj->send, (uint8_t *)&(obj->send_data));  // 板间通信
}

void Up_and_Down_z_Update(Gripper *obj) {
    if (obj->remote->data.rc.ch4 > 1100)
        servo_1_target_degree += 0.05;
    else if (obj->remote->data.rc.ch4 < 900)
        servo_1_target_degree -= 0.05;
    if (servo_1_target_degree > 175) {
        servo_1_target_degree = 175;
    } else if (servo_1_target_degree < 95) {
        servo_1_target_degree = 95;
    }
    obj->servo_1->pos_servo_control = servo_1_target_degree;
}
void Fowardback_and_Roll_xy_Update(Gripper *obj) {
    switch (obj->mode) {
        case stop:
        case reset:
            motor1_spd = 0;
            motor2_spd = 0;
            break;
        case run:
            motor1_spd = (obj->remote->data.rc.ch3 - 1024) + (1024 - obj->remote->data.rc.ch2);
            motor2_spd = (obj->remote->data.rc.ch3 - 1024) + (obj->remote->data.rc.ch2 - 1024);
            break;
    }
    Motor1_Setspd(motor1_spd, obj);
    Motor2_Setspd(motor2_spd, obj);
}
void Clamp_Update(Gripper *obj) {  //夹爪抓取放下
    // static float servo_2_angle = 5, servo_3_angle = 180;
    // // servo_2_angle += ((float)obj->remote->data.rc.ch1 - 1024.0) ;
    // // servo_3_angle -= ((float)obj->remote->data.rc.ch1 - 1024.0) ;
    // servo_2_angle = abs((float)obj->remote->data.rc.ch1 - 1024.0) ;
    // servo_3_angle = 180-abs((float)obj->remote->data.rc.ch1 - 1024.0) ;
    // if (servo_2_angle < 5)
    //     servo_2_angle = 5;
    // else if (servo_2_angle > 170)
    //     servo_2_angle = 170;
    // if (servo_3_angle < 0)
    //     servo_3_angle = 0;
    // else if (servo_3_angle > 180)
    //     servo_3_angle = 180;

    // obj->servo_2->pos_servo_control = servo_2_angle;
    // obj->servo_3->pos_servo_control = servo_3_angle;
}
void Side_Updown(Gripper *obj) {  //侧面机构(带着电机的)抬升和下降
    obj->send_data.servo6_pos = (int)(((float)obj->remote->data.rc.ch0 - 1024.0) * (66.0 / 660.0) + 100.0);
    obj->send_data.servo7_pos = (int)((1024.0 - (float)obj->remote->data.rc.ch0) * (66.0 / 660.0) + 55.0);
}

void Mode_Update(Gripper *obj) {
    switch (obj->remote->data.rc.s2) {
        case 1:
            obj->mode = reset;
            break;
        case 3:
            obj->mode = stop;
            break;
        case 2:
            obj->mode = run;
            break;
        default:
            break;
    }
}
void Motor1_Setspd(int spd, Gripper *obj) {
    if (spd == 0) {
        obj->motor_1_positive->pos_servo_control = -45;  // 1755
        obj->motor_1_nagitive->pos_servo_control = -45;
    } else if (spd > 0) {
        obj->motor_1_positive->pos_servo_control = spd;
        obj->motor_1_nagitive->pos_servo_control = -45;
    } else if (spd < 0) {
        obj->motor_1_positive->pos_servo_control = -45;
        obj->motor_1_nagitive->pos_servo_control = -spd;
    }
}
void Motor2_Setspd(int spd, Gripper *obj) {
    if (spd == 0) {
        obj->motor_2_positive->pos_servo_control = -45;
        obj->motor_2_nagitive->pos_servo_control = -45;
    } else if (spd > 0) {
        obj->motor_2_positive->pos_servo_control = spd;
        obj->motor_2_nagitive->pos_servo_control = -45;
    } else if (spd < 0) {
        obj->motor_2_positive->pos_servo_control = -45;
        obj->motor_2_nagitive->pos_servo_control = -spd;
    }
}

#else
void Gripper_Update(Gripper *obj) {
    if (obj->imu->bias_init_success) {
        if (!buzzer_started) {
            buzzer_started = 1;
            Buzzer_Start(obj->internal_buzzer);
        }
    }

    if (obj->recv->data_updated) {
        obj->servo_6->pos_servo_control = obj->recv_data->servo6_pos;
        obj->servo_7->pos_servo_control = obj->recv_data->servo7_pos;
    } else {
        obj->servo_6->pos_servo_control = 100;
        obj->servo_7->pos_servo_control = 55;
    }

    obj->send_data.init_flag = obj->imu->bias_init_success;
    obj->send_data.euler_x = obj->imu->data.euler[0];
    obj->send_data.euler_y = obj->imu->data.euler[1];
    obj->send_data.euler_z = obj->imu->data.euler[2];
    CanSend_Send(obj->send, (uint8_t *)&(obj->send_data));
}
#endif