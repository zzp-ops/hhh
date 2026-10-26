/**
  ******************************************************************************
  * @file    Motor.c
  * @brief   直流减速电机 + TB6612 驱动层
  *
  * 两件事：
  *   1. 一路 PWM (PA8/TIM1_CH1) + 两个方向脚 (PB14/PB15) 控制转速和转向
  *   2. TIM4 硬件编码器读电机转角
  *
  * 硬件细节和方向真值表见 Motor.h 开头。
  ******************************************************************************
  */

#include "Motor.h"

/* 来自 main.c 的 CubeMX 生成句柄 */
extern TIM_HandleTypeDef htim1;
extern TIM_HandleTypeDef htim4;

/* ---------------------------------------------------------------------------
 * 内部状态
 * ------------------------------------------------------------------------- */

static int16_t s_last_encoder = 0;

/* 读一次编码器计数。
 * TIM4 是 16 位计数器，硬件会根据 A/B 相自动增减，这里只是取出来。
 * 转成 int16_t 之后，0xFFFF 会被解释成 -1，正好符合「反转」的语义。        */
static int16_t motor_read_encoder(void)
{
    return (int16_t)__HAL_TIM_GET_COUNTER(&htim4);
}

/* 设置方向脚。dir > 0 正转，dir < 0 反转，dir == 0 滑行。 */
static void motor_set_direction(int8_t dir)
{
    if (dir > 0) {
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    } else if (dir < 0) {
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_RESET);
    }
}

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */

void Motor_Init(void)
{
    /* 先把方向脚拉到「滑行」，PWM 占空比清零，避免上电瞬间电机乱动 */
    motor_set_direction(0);
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);

    /* 启动 PWM 输出。TIM1 是高级定时器，HAL 会顺带打开 MOE（主输出使能），
     * 不打开的话即使 CCR 有值，PA8 也不会有波形。                          */
    (void)HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);

    /* 启动编码器接口，TIM4 开始根据 PB6/PB7 上的 A/B 相自动增减计数 */
    (void)HAL_TIM_Encoder_Start(&htim4, TIM_CHANNEL_ALL);

    Motor_ResetEncoder();
}

/* ---------------------------------------------------------------------------
 * 功率输出
 * ------------------------------------------------------------------------- */

void Motor_SetPower(int16_t power)
{
    int32_t  p = power;
    uint32_t ccr;

    /* 限幅 */
    if (p > MOTOR_POWER_MAX) {
        p = MOTOR_POWER_MAX;
    } else if (p < -MOTOR_POWER_MAX) {
        p = -MOTOR_POWER_MAX;
    }

#if MOTOR_INVERT
    p = -p;
#endif

    /* 先定方向，再写占空比。
     * TB6612 内部有死区逻辑，不会因为 IN1/IN2 同时切换而直通。             */
    if (p > 0) {
        motor_set_direction(1);
    } else if (p < 0) {
        motor_set_direction(-1);
        p = -p;
    } else {
        motor_set_direction(0);
    }

    /* 功率量程映射到 PWM 计数：
     * p = 1000  → ccr = 3600 = ARR+1 = 100% 占空比
     * p = 1     → ccr = 3    = 约 0.08% 占空比（TB6612 可能转不起来但这是正常的）*/
    ccr = ((uint32_t)p * (MOTOR_PWM_PERIOD + 1U)) / (uint32_t)MOTOR_POWER_MAX;

    if (ccr > (MOTOR_PWM_PERIOD + 1U)) {
        ccr = MOTOR_PWM_PERIOD + 1U;
    }

    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, ccr);
}

void Motor_Stop(void)
{
    /* 高阻滑行：占空比清零 + 两个方向脚都为低 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    motor_set_direction(0);
}

void Motor_Brake(void)
{
    /* 刹车：占空比清零 + BIN1/BIN2 同时为高，短接电机绕组 */
    __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_1, 0);
    HAL_GPIO_WritePin(MOTOR_BIN1_PORT, MOTOR_BIN1_PIN, GPIO_PIN_SET);
    HAL_GPIO_WritePin(MOTOR_BIN2_PORT, MOTOR_BIN2_PIN, GPIO_PIN_SET);
}

/* ---------------------------------------------------------------------------
 * 编码器
 * ------------------------------------------------------------------------- */

int16_t Motor_GetEncoder(void)
{
    return motor_read_encoder();
}

int16_t Motor_GetEncoderDelta(void)
{
    int16_t now;
    int16_t delta;

    now   = motor_read_encoder();
    delta = (int16_t)(now - s_last_encoder);

    s_last_encoder = now;

    return delta;
}

void Motor_ResetEncoder(void)
{
    __HAL_TIM_SET_COUNTER(&htim4, 0);
    s_last_encoder = 0;
}
