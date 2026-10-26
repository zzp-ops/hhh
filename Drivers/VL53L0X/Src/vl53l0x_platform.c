/**
  ******************************************************************************
  * @file    vl53l0x_platform.c
  * @brief   ST VL53L0X API 的平台层（STM32 移植版）
  *
  * 对应官方包 Api/platform/src/vl53l0x_platform.c。和官方版本的差别只有三处：
  *
  *   1) 去掉 <Windows.h>
  *      VL53L0X_PollingDelay() 原本用 WaitForSingleObject 等 1ms，
  *      这里换成 HAL_Delay(1)。
  *
  *   2) 底层收发不再走 RANGING_SENSOR_COMMS_xxx（那是 USB 转 I2C 小板的接口），
  *      改成调用 vl53l0x_i2c_platform.c 里的 VL53L0X_write_multi / read_multi，
  *      最终落到 HAL_I2C_Mem_Write / HAL_I2C_Mem_Read。
  *
  *   3) 不再使用 LOG_FUNCTION_START / LOG_FUNCTION_END
  *      这两个宏在没定义 VL53L0X_LOG_ENABLE 时展开成 (void)0，留着也没用，
  *      直接删掉让代码更干净。
  *
  * 除了以上三点，寄存器封装的逻辑（尤其是大小端和 And/Or 更新）与官方逐行一致，
  * 不要随手改，改了 API 内部的读写就对不上了。
  ******************************************************************************
  */

#include "vl53l0x_platform.h"
#include "vl53l0x_i2c_platform.h"
#include "vl53l0x_api.h"
#include "main.h"

/* I2C 单次传输的最大字节数。官方取 64，够 API 内部所有批量读用。 */
#define VL53L0X_MAX_I2C_XFER_SIZE   64

/* ---------------------------------------------------------------------------
 * 总线互斥
 * 本工程只有一颗 VL53L0X，也没有 RTOS，所以这两个函数保持空实现。
 * 以后如果要挂两颗（靠 XSHUT 分时改地址），就在这里加临界区。
 * ------------------------------------------------------------------------- */

VL53L0X_Error VL53L0X_LockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_UnlockSequenceAccess(VL53L0X_DEV Dev)
{
    (void)Dev;
    return VL53L0X_ERROR_NONE;
}

/* ---------------------------------------------------------------------------
 * 多字节读写
 * ------------------------------------------------------------------------- */

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    if (VL53L0X_write_multi(Dev->I2cDevAddr, index, pdata, (int32_t)count) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_DEV Dev, uint8_t index, uint8_t *pdata, uint32_t count)
{
    if (count >= VL53L0X_MAX_I2C_XFER_SIZE) {
        return VL53L0X_ERROR_INVALID_PARAMS;
    }

    if (VL53L0X_read_multi(Dev->I2cDevAddr, index, pdata, (int32_t)count) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

/* ---------------------------------------------------------------------------
 * 单字节 / 字 / 双字 读写
 * ------------------------------------------------------------------------- */

VL53L0X_Error VL53L0X_WrByte(VL53L0X_DEV Dev, uint8_t index, uint8_t data)
{
    if (VL53L0X_write_byte(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrWord(VL53L0X_DEV Dev, uint8_t index, uint16_t data)
{
    if (VL53L0X_write_word(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_WrDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t data)
{
    if (VL53L0X_write_dword(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdByte(VL53L0X_DEV Dev, uint8_t index, uint8_t *data)
{
    if (VL53L0X_read_byte(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdWord(VL53L0X_DEV Dev, uint8_t index, uint16_t *data)
{
    if (VL53L0X_read_word(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

VL53L0X_Error VL53L0X_RdDWord(VL53L0X_DEV Dev, uint8_t index, uint32_t *data)
{
    if (VL53L0X_read_dword(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

/**
  * @brief  读-改-写单个寄存器
  * @note   最终值 = (原值 & AndData) | OrData，与官方实现一致
  */
VL53L0X_Error VL53L0X_UpdateByte(VL53L0X_DEV Dev, uint8_t index, uint8_t AndData, uint8_t OrData)
{
    uint8_t data = 0;

    if (VL53L0X_read_byte(Dev->I2cDevAddr, index, &data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    data = (uint8_t)((data & AndData) | OrData);

    if (VL53L0X_write_byte(Dev->I2cDevAddr, index, data) != 0) {
        return VL53L0X_ERROR_CONTROL_INTERFACE;
    }

    return VL53L0X_ERROR_NONE;
}

/**
  * @brief  API 内部轮询时插入的短延时
  * @note   VL53L0X_device_read_strobe() 之类的地方，每写一个 strobe 后要等一小会儿
  *         让固件把数据搬到 0x90 寄存器。官方 Windows 版是等 1ms 的事件对象，
  *         这里直接 HAL_Delay(1)。
  *
  *         注意：这个函数会被 API 在「上电 + 初始化 + 每次单次测距」中反复调用，
  *         累计起来几百毫秒是正常的，不要把测量放进对时间要求极严的中断里。
  */
VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_DEV Dev)
{
    (void)Dev;
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}
