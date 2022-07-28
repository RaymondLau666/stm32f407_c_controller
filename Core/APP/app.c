#include "app.h"


#include "gripper.h"
#include "gripper_board_cmd.h"

gripper_board_cmd* cmd;
Gripper* gripper;
void APP_Layer_Init(){
    cmd = Gripper_board_CMD_Create();
    gripper = Gripper_Create();
}

void APP_Loop() {
    Gripper_board_CMD_Update(cmd);
    Gripper_Update(gripper);
}

// 打印输出等到ozone的窗口 用于测试项目
void APP_Log_Loop() {}

void APP_Layer_default_loop() {
    // if (gripper->imu->bias_init_success) {
    //     Buzzer_Update(internal_buzzer);
    // }
}
