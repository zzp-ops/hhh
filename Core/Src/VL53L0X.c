/**
  ******************************************************************************
  * @file    VL53L0X.c
  * @brief   VL53L0X 激光测距驱动 —— 工程封装层（非阻塞版）
  *
  * 这一层把 ST 官方 API 那一长串初始化收成一个 VL53L0X_Init()，
  * 再把测距拆成一个不阻塞的状态机：
  *
  *     IDLE --StartSingleShot()--> WAIT --PollMeasDone()==1--> 取数 --> IDLE
  *
  * 官方那个 VL53L0X_PerformSingleRangingMeasurement() 内部要死等满
  * timing budget（默认 33ms），本文件故意不用它。
  *
  * 官方 API 本身的寄存器时序、调参表、SPAD 校准、测距状态机都在
  * Core/Src/vl53l0x 下，本文件一行都没有重写，只是调用。
  *
  * 接线与用法见 VL53L0X.h 开头。
  ******************************************************************************
  */

#include "VL53L0X.h"

/* ---------------------------------------------------------------------------
 * 可调项
 * ------------------------------------------------------------------------- */

/* 是否每次上电都跑一次「参考 SPAD 校准」。
 *
 * 保持 1：第一次调试用这个，最稳妥，每次上电都按当前环境重新标定。
 *        代价是启动多花大约 1 秒（只在 main 里跑一次，不影响实时性）。
 * 改成 0：跑通之后可以关掉。校准结果 ST 已经写进芯片自己的 NVM，
 *        VL53L0X_StaticInit() 会自动读出来用，启动时间降到 100ms 以内。            */
#ifndef VL53L0X_RUN_REF_SPAD_MANAGEMENT
#define VL53L0X_RUN_REF_SPAD_MANAGEMENT     1
#endif

/* 单次测距默认时间预算，33ms 是 ST 的推荐值。
 * 想出速度就往下调到 20000，想更稳就往上调。                                    */
#define VL53L0X_DEFAULT_TIMING_BUDGET_US    33000U

/* 超时保护余量：预算时间过了这么久还没报完成，就认为芯片卡住了。
 * 正常情况下测量在预算时刻前后就会完成，留 20ms 足够宽松。                     */
#define VL53L0X_TIMEOUT_MARGIN_MS           20U

/* ---------------------------------------------------------------------------
 * 内部状态
 * ------------------------------------------------------------------------- */

static VL53L0X_Dev_t s_dev;                        /* ST API 的设备句柄         */
static uint8_t       s_ready             = 0;      /* 初始化是否成功            */
static uint8_t       s_state             = VL53L0X_ST_IDLE;
static uint32_t      s_start_tick        = 0;      /* StartSingleShot 的时刻    */
static uint32_t      s_wait_ms           = VL53L0X_DEFAULT_TIMING_BUDGET_US / 1000U;
static uint8_t       s_range_status      = 0xFF;   /* 最近一次的测距状态        */
static uint16_t      s_signal_rate_mcps  = 0;      /* 最近一次的回波强度 MCPS   */
static uint32_t      s_timing_budget_us  = VL53L0X_DEFAULT_TIMING_BUDGET_US;

/* 官方 API 有个「校准警告」(-1) 不是致命错误，允许它过去 */
static uint8_t vl53l0x_ok(VL53L0X_Error status)
{
    return (uint8_t)((status == VL53L0X_ERROR_NONE) ||
                     (status == VL53L0X_ERROR_CALIBRATION_WARNING));
}

/* ---------------------------------------------------------------------------
 * 初始化
 * ------------------------------------------------------------------------- */

uint8_t VL53L0X_Init(I2C_HandleTypeDef *hi2c)
{
    VL53L0X_Error status;
    uint8_t  vhv_settings      = 0;
    uint8_t  phase_cal         = 0;
    uint8_t  is_aperture_spads = 0;
    uint32_t ref_spad_count    = 0;
    uint8_t  model_id          = 0;

    s_ready            = 0;
    s_state            = VL53L0X_ST_IDLE;
    s_range_status     = 0xFF;
    s_signal_rate_mcps = 0;

    if (hi2c == NULL) {
        return 0;
    }

    /* 把 I2C 句柄交给底层（vl53l0x_i2c_platform.c） */
    VL53L0X_I2C_Bind(hi2c);

    /* --- 1. 复位时序 -------------------------------------------------------
     * XSHUT 拉低让芯片进入 Hw Standby，再拉高让它从 Flash 启动固件。
     * 固件启动约 1.2ms，这里等 2ms 留足余量。这一步在上电初始化里，
     * 不属于控制环，阻塞是安全的。                                          */
    HAL_GPIO_WritePin(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN, GPIO_PIN_RESET);
    HAL_Delay(2);
    HAL_GPIO_WritePin(VL53L0X_XSHUT_PORT, VL53L0X_XSHUT_PIN, GPIO_PIN_SET);
    HAL_Delay(2);

    /* --- 2. 设备句柄 ------------------------------------------------------
     * 字段含义同 ST 例程：I2cDevAddr 用左移后的 8 位地址 0x52。             */
    s_dev.I2cDevAddr      = VL53L0X_I2C_ADDR;
    s_dev.comms_type      = 1;      /* 1 = I2C */
    s_dev.comms_speed_khz = 400;

    /* --- 3. 确认芯片在线 --------------------------------------------------
     * 读型号寄存器 0xC0，VL53L0X 固定返回 0xEE。读不到或者值不对，
     * 说明没接上、供电有问题，或者 XSHUT 没被拉起来，直接判失败。            */
    if (VL53L0X_RdByte(&s_dev, VL53L0X_MODEL_ID_REG, &model_id) != VL53L0X_ERROR_NONE) {
        return 0;
    }
    if (model_id != VL53L0X_MODEL_ID_VALUE) {
        return 0;
    }

    /* --- 4. DataInit：基本寄存器复位 --------------------------------------
     * 会把 PAL 状态机推到 WAIT_STATICINIT，并打开全部检查项、再把其中
     * 四项关掉。这一步是后面所有步骤的前提。                                */
    status = VL53L0X_DataInit(&s_dev);
    if (!vl53l0x_ok(status)) {
        return 0;
    }

    /* --- 5. StaticInit：SPAD、调参表、中断配置、时序 ----------------------
     * 里面会读芯片 NVM 拿出厂标定的参考 SPAD 数量和类型，然后整张
     * DefaultTuningSettings 调参表逐条写下去。这张表不能改。                */
    status = VL53L0X_StaticInit(&s_dev);
    if (!vl53l0x_ok(status)) {
        return 0;
    }

    /* --- 6. 参考校准（温度/VHV 与相位）------------------------------------
     * 输出两个和环境温度相关的参数。环境温差超过 8 度时需要重跑。           */
    status = VL53L0X_PerformRefCalibration(&s_dev, &vhv_settings, &phase_cal);
    if (!vl53l0x_ok(status)) {
        return 0;
    }

    /* --- 7. 参考 SPAD 校准 -------------------------------------------------
     * 决定要启用多少个参考 SPAD 才能对目标测得最好。结果会写进芯片 NVM。
     *
     * 这一步如果传感器正前方有高反射物体（比如水管直接怼在镜头上），
     * 会返回 -50。这里故意不判死：StaticInit() 已经用 NVM 里出厂标定的
     * SPAD 值配好了，测距照样能用，只是不是当前环境最优。                   */
#if VL53L0X_RUN_REF_SPAD_MANAGEMENT
    (void)VL53L0X_PerformRefSpadManagement(&s_dev, &ref_spad_count, &is_aperture_spads);
#else
    (void)ref_spad_count;
    (void)is_aperture_spads;
#endif

    /* --- 8. 单次测距模式 -------------------------------------------------- */
    status = VL53L0X_SetDeviceMode(&s_dev, VL53L0X_DEVICEMODE_SINGLE_RANGING);
    if (!vl53l0x_ok(status)) {
        return 0;
    }

    /* --- 9. 打开三项检查（与 ST 单次测距例程一致）-------------------------
     * SIGMA_FINAL_RANGE       精度不够的测量直接判无效
     * SIGNAL_RATE_FINAL_RANGE 回波太弱的测量判无效
     * RANGE_IGNORE_THRESHOLD  丢弃低于阈值的近距离串扰（下面设 1.5*0.023）   */
    (void)VL53L0X_SetLimitCheckEnable(&s_dev, VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE, 1);
    (void)VL53L0X_SetLimitCheckEnable(&s_dev, VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE, 1);
    (void)VL53L0X_SetLimitCheckEnable(&s_dev, VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD, 1);
    (void)VL53L0X_SetLimitCheckValue(&s_dev,
                                     VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD,
                                     (FixPoint1616_t)(1.5 * 0.023 * 65536));

    /* --- 10. 时间预算 ----------------------------------------------------- */
    s_ready = 1;
    VL53L0X_SetTimingBudgetUs(VL53L0X_DEFAULT_TIMING_BUDGET_US);

    return 1;
}

uint8_t VL53L0X_IsReady(void)
{
    return s_ready;
}

uint8_t VL53L0X_GetState(void)
{
    return s_state;
}

/* ---------------------------------------------------------------------------
 * 非阻塞测距：三步走
 * ------------------------------------------------------------------------- */

uint8_t VL53L0X_StartSingleShot(void)
{
    if (!s_ready) {
        return 0;
    }

    /* 上一次的结果还没被取走，不允许覆盖 */
    if (s_state != VL53L0X_ST_IDLE) {
        return 0;
    }

    /* 官方 VL53L0X_StartMeasurement()：
     * 几笔寄存器写 + 有界轮询等 START 位清零，内部没有 HAL_Delay。
     * 正常情况下 1~2 次读就能看到 START 位清掉，耗时约 0.2ms。             */
    if (VL53L0X_StartMeasurement(&s_dev) != VL53L0X_ERROR_NONE) {
        s_state = VL53L0X_ST_IDLE;
        return 0;
    }

    s_start_tick = HAL_GetTick();
    s_state      = VL53L0X_ST_WAIT;

    return 1;
}

int8_t VL53L0X_PollMeasDone(void)
{
    uint8_t  ready = 0;
    uint32_t elapsed_ms;

    if (!s_ready) {
        return VL53L0X_ERR_NOT_INIT;
    }

    if (s_state != VL53L0X_ST_WAIT) {
        return VL53L0X_ERR_NOT_STARTED;
    }

    /* uint32_t 相减，HAL_GetTick() 回绕时结果依然正确 */
    elapsed_ms = HAL_GetTick() - s_start_tick;

    /* 还没到预算时间，芯片肯定没测完。这里直接返回，一次 I2C 都不发，
     * 这就是本驱动在控制环里几乎不占时间的原因。                            */
    if (elapsed_ms < s_wait_ms) {
        return 0;
    }

    /* 超时保护：预算 + 余量都过了还没完成，认为芯片卡住，复位状态机。
     * 调用方看到 VL53L0X_ERR_TIMEOUT 后重新 StartSingleShot() 即可恢复。   */
    if (elapsed_ms > (s_wait_ms + VL53L0X_TIMEOUT_MARGIN_MS)) {
        s_state = VL53L0X_ST_IDLE;
        return VL53L0X_ERR_TIMEOUT;
    }

    /* 到点了，问芯片一句「好了没」。这是一次 1 字节的寄存器读，约 0.05ms。 */
    if (VL53L0X_GetMeasurementDataReady(&s_dev, &ready) != VL53L0X_ERROR_NONE) {
        s_state = VL53L0X_ST_IDLE;
        return VL53L0X_ERR_API;
    }

    return (int8_t)(ready ? 1 : 0);
}

int32_t VL53L0X_GetDistanceMm(void)
{
    VL53L0X_RangingMeasurementData_t data;
    VL53L0X_Error status;

    if (!s_ready) {
        return VL53L0X_ERR_NOT_INIT;
    }

    if (s_state != VL53L0X_ST_WAIT) {
        return VL53L0X_ERR_NOT_STARTED;
    }

    /* 读 0x14 开始的 12 字节结果区（信号强度、环境光、SPAD 数、距离、状态） */
    status = VL53L0X_GetRangingMeasurementData(&s_dev, &data);

    /* 无论读成功与否都要清中断，否则下一次测量永远不会报完成 */
    (void)VL53L0X_ClearInterruptMask(&s_dev, 0);

    s_state = VL53L0X_ST_IDLE;

    if (status != VL53L0X_ERROR_NONE) {
        return VL53L0X_ERR_API;
    }

    s_range_status = data.RangeStatus;

    /* SignalRateRtnMegaCps 是 16.16 定点，右移 16 位得到整数 MCPS */
    s_signal_rate_mcps = (uint16_t)(data.SignalRateRtnMegaCps >> 16);

    /* RangeStatus 非 0 说明这次测量被芯片自己的检查判为无效，
     * 这时候 RangeMilliMeter 里的数字不可信，不能拿去喂 PID。 */
    if (data.RangeStatus != 0) {
        return VL53L0X_ERR_RANGE;
    }

    return (int32_t)data.RangeMilliMeter;
}

/* ---------------------------------------------------------------------------
 * 非阻塞测距：一行流
 * ------------------------------------------------------------------------- */

int8_t VL53L0X_Service(int32_t *distance_mm)
{
    int8_t poll_result;

    if (!s_ready) {
        return VL53L0X_ERR_NOT_INIT;
    }

    /* 空闲：开一次新测量，本周期没有数据可给 */
    if (s_state == VL53L0X_ST_IDLE) {
        if (!VL53L0X_StartSingleShot()) {
            return VL53L0X_ERR_API;
        }
        return 0;
    }

    /* 测量中：问一下好了没 */
    poll_result = VL53L0X_PollMeasDone();

    if (poll_result < 0) {
        /* 出错或超时：状态机已经被复位成 IDLE，下一周期会自动重新启动 */
        return poll_result;
    }

    if (poll_result == 0) {
        /* 还没好，本周期什么都不做 */
        return 0;
    }

    /* 好了，取结果。取完状态机自动回到 IDLE，下次调用会开新的一轮。 */
    if (distance_mm != NULL) {
        *distance_mm = VL53L0X_GetDistanceMm();
    } else {
        (void)VL53L0X_GetDistanceMm();
    }

    return 1;
}

/* ---------------------------------------------------------------------------
 * 结果与诊断
 * ------------------------------------------------------------------------- */

uint8_t VL53L0X_GetRangeStatus(void)
{
    return s_range_status;
}

uint16_t VL53L0X_GetSignalRateMcps(void)
{
    return s_signal_rate_mcps;
}

/* ---------------------------------------------------------------------------
 * 参数调整
 * ------------------------------------------------------------------------- */

void VL53L0X_SetOffsetMm(int16_t offset_mm)
{
    if (!s_ready) {
        return;
    }

    /* API 要的单位是微米 */
    (void)VL53L0X_SetOffsetCalibrationDataMicroMeter(&s_dev, (int32_t)offset_mm * 1000);
}

int16_t VL53L0X_GetOffsetMm(void)
{
    int32_t offset_um = 0;

    if (!s_ready) {
        return 0;
    }

    if (VL53L0X_GetOffsetCalibrationDataMicroMeter(&s_dev, &offset_um) != VL53L0X_ERROR_NONE) {
        return 0;
    }

    return (int16_t)(offset_um / 1000);
}

void VL53L0X_SetTimingBudgetUs(uint32_t us)
{
    if (!s_ready) {
        /* 还没初始化就先记着，等 Init 里 s_ready 置位后再真正下发 */
        s_timing_budget_us = us;
        s_wait_ms          = us / 1000U;
        return;
    }

    if (VL53L0X_SetMeasurementTimingBudgetMicroSeconds(&s_dev, us) == VL53L0X_ERROR_NONE) {
        s_timing_budget_us = us;

        /* PollMeasDone() 用这个值判断「还不到点，别浪费 I2C」，
         * 所以必须跟着预算一起更新，否则改了预算等待时间还是旧的。          */
        s_wait_ms = us / 1000U;
    }
}

uint32_t VL53L0X_GetTimingBudgetUs(void)
{
    uint32_t us = 0;

    if (s_ready &&
        (VL53L0X_GetMeasurementTimingBudgetMicroSeconds(&s_dev, &us) == VL53L0X_ERROR_NONE)) {
        s_timing_budget_us = us;
        s_wait_ms          = us / 1000U;
    }

    return s_timing_budget_us;
}

/* ---------------------------------------------------------------------------
 * 逃生口
 * ------------------------------------------------------------------------- */

VL53L0X_DEV VL53L0X_GetDevice(void)
{
    return &s_dev;
}
