#include "gripper_board_cmd.h"

#include "bsp.h"

// monitor处理函数
void gripper_core_module_lost(void* obj) {
    // 暂时仅调试
    printf_log("gripper_core_module_lost!!!robot stopped for security.\n");
}

gripper_board_cmd* Gripper_board_CMD_Create() {
    // 创建实例
    gripper_board_cmd* obj = (gripper_board_cmd*)malloc(sizeof(gripper_board_cmd));
    memset(obj, 0, sizeof(gripper_board_cmd));
    return obj;
}

void Gripper_board_CMD_Update(gripper_board_cmd* obj) {

}