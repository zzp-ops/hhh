/**
  ******************************************************************************
  * @file    GetData.c
  * @brief   测距数据获取层
  *
  * 这一层只负责两件事：
  *   1. 把 VL53L0X 初始化一次，并记录「模块启动状态」
  *   2. 非阻塞地读距离，单位 mm
  *
  * 下面压着两个东西：
  *   Core/Src/VL53L0X.c        工程封装层（非阻塞状态机）
  *   Drivers/VL53L0X/          ST 官方 API + STM32 平台层
  *
  * 所以这里不需要再关心寄存器、大端、I2C 时序，只管把结果给上层。
  ******************************************************************************
  */

#include "GetData.h"
#include "VL53L0X.h"

/* ---------------------------------------------------------------------------
 * 内部状态
 * ------------------------------------------------------------------------- */

static GetData_State s_state         = GETDATA_STATE_UNINIT;
static uint8_t       s_init_step     = 0;
static int32_t       s_last_distance = GETDATA_NO_DATA;

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */

GetData_State GetData_Init(I2C_HandleTypeDef *hi2c)
{
    s_last_distance = GETDATA_NO_DATA;
    s_init_step     = 0;
    s_state         = GETDATA_STATE_UNINIT;

    if (VL53L0X_Init(hi2c) != 0) {
        s_state = GETDATA_STATE_OK;
        return s_state;
    }

    /* 初始化失败，用 VL53L0X 给出的步骤号区分是硬件问题还是软件配置问题 */
    s_init_step = VL53L0X_GetInitError();

    if ((s_init_step == 2) || (s_init_step == 3)) {
        /* 2 = 读型号寄存器无应答 → 线没接好、没供电、上拉缺失、XSHUT 没拉高
         * 3 = 型号不是 0xEE      → 接的不是 VL53L0X，或者芯片没启动成功     */
        s_state = GETDATA_STATE_NODEV;
    } else {
        /* 4~7 = DataInit / StaticInit / 参考校准 / 设模式 失败
         * 芯片在线能通信，问题出在配置阶段，多半是通信中途出错              */
        s_state = GETDATA_STATE_FAIL;
    }

    return s_state;
}

GetData_State GetData_GetState(void)
{
    return s_state;
}

uint8_t GetData_IsReady(void)
{
    return (uint8_t)(s_state == GETDATA_STATE_OK);
}

uint8_t GetData_GetInitStep(void)
{
    return s_init_step;
}

/* ---------------------------------------------------------------------------
 * 读距离
 * ------------------------------------------------------------------------- */

int32_t GetData_ReadDistanceMm(void)
{
    int32_t mm = 0;
    int8_t  serviced;

    if (s_state != GETDATA_STATE_OK) {
        return GETDATA_UNINIT;
    }

    /* 非阻塞推进：内部自己管「启动 / 等 / 取 / 再启动」，
     * 没拿到新样本时返回 0，不会卡住调用者。                              */
    serviced = VL53L0X_Service(&mm);

    if (serviced == 1) {
        /* 这一次拿到了新样本 */
        if (mm >= 0) {
            s_last_distance = mm;
        } else {
            /* 测到了但芯片判为无效：没目标、回波太弱、精度不够 */
            s_last_distance = GETDATA_INVALID;
        }
    } else if (serviced < 0) {
        /* 超时或者通信出错。状态机已经被 VL53L0X 层复位，
         * 下一次调用会自动重新开始一轮，这里只需要把对外结果标成无效。      */
        s_last_distance = GETDATA_INVALID;
    }

    return s_last_distance;
}
