#ifndef _Gripper_BOARD_CMD_H_
#define _Gripper_BOARD_CMD_H_

#include "pub_sub.h"
#include "stdint.h"
// 调试
#include "bsp_log.h"
// 外设
#include "BMI088.h"
#include "buzzer.h"
#include "can_pc.h"
#include "can_recv.h"
#include "can_send.h"
#include "referee.h"
#include "indicator_led.h"

#pragma pack(1)
typedef struct Gripper_board_cmd_t {

} gripper_board_cmd;
#pragma pack()

gripper_board_cmd *Gripper_board_CMD_Create(void);
void Gripper_board_CMD_Update(gripper_board_cmd *obj);

#endif