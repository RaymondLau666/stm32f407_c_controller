#include "bsp_pwm.h"

#include "bsp_def.h"
#include "dma.h"
#include "tim.h"

typedef struct BSP_PWM_Typedef_t {
    TIM_HandleTypeDef* base;
    uint16_t channel;
} BSP_PWM_Typedef;

BSP_PWM_Typedef pwm_ports[DEVICE_PWM_CNT];
extern DMA_HandleTypeDef PWM_DMA_1;
void BSP_PWM_Init() {
    pwm_ports[0].base = PWM_0_BASE;
    pwm_ports[0].channel = PWM_0_CHANNEL;

    pwm_ports[1].base = PWM_1_BASE;
    pwm_ports[1].channel = PWM_1_CHANNEL;

    pwm_ports[2].base = PWM_2_BASE;
    pwm_ports[2].channel = PWM_2_CHANNEL;

    pwm_ports[3].base = PWM_3_BASE;
    pwm_ports[3].channel = PWM_3_CHANNEL;

    pwm_ports[4].base = PWM_4_BASE;
    pwm_ports[4].channel = PWM_4_CHANNEL;

    pwm_ports[5].base = PWM_5_BASE;
    pwm_ports[5].channel = PWM_5_CHANNEL;

    pwm_ports[6].base = PWM_6_BASE;
    pwm_ports[6].channel = PWM_6_CHANNEL;

    pwm_ports[7].base = PWM_7_BASE;
    pwm_ports[7].channel = PWM_7_CHANNEL;

    pwm_ports[8].base = PWM_8_BASE;
    pwm_ports[8].channel = PWM_8_CHANNEL;
}

void BSP_PWM_Start(uint8_t pwm_index) { HAL_TIM_PWM_Start(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel); }

void BSP_PWM_Stop(uint8_t pwm_index) { HAL_TIM_PWM_Stop(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel); }

void BSP_PWM_SetCCR(uint8_t pwm_index, uint32_t ccr_value) { __HAL_TIM_SetCompare(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel, ccr_value); }

void BSP_PWM_SetARR(uint8_t pwm_index, uint32_t arr_value) { __HAL_TIM_SetAutoreload(pwm_ports[pwm_index].base, arr_value); }

void BSP_PWM_StartCCR_DMA(uint8_t pwm_index, uint32_t* ccr_data, uint16_t len) { HAL_TIM_PWM_Start_DMA(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel, ccr_data, len); }

uint32_t BSP_PWM_ReadCCR(uint8_t pwm_index) { return HAL_TIM_ReadCapturedValue(pwm_ports[pwm_index].base, pwm_ports[pwm_index].channel); }