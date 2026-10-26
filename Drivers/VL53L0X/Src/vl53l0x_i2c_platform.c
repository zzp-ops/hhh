/**
  ******************************************************************************
  * @file    vl53l0x_i2c_platform.c
  * @brief   VL53L0X 官方 API 的 I2C 底层移植（STM32 HAL）
  *
  * 官方 API 的收发是两层：
  *
  *   vl53l0x_platform.c     寄存器级封装（WrByte / RdWord / UpdateByte ...），与平台无关
  *   vl53l0x_i2c_platform.c 裸 I2C 收发，就是本文件
  *
  * ST 官方包里这一层是写给「电脑 USB 转 I2C 小板」用的（RANGING_SENSOR_COMMS_xxx），
  * 单片机根本用不上，所以整个换成 HAL_I2C_Mem_Read / HAL_I2C_Mem_Write。
  *
  * 关于地址：ST 例程里统一写 Dev->I2cDevAddr = 0x52，也就是 7 位地址 0x29 左移一位
  * 后、最低位当读写标志的那个 8 位形式。HAL 的 DevAddress 参数要的正好也是这个
  * 已经左移好的 8 位地址，由 HAL 自己去置最低位，所以这里直接透传，不用再换算。
  *
  * 关于字节序：VL53L0X 的寄存器是**大端**（高字节在前）。write_word / read_word /
  * write_dword / read_dword 的拼装顺序必须和官方实现一致，否则 API 读到的时间预算、
  * SPAD 信息全是错的。
  ******************************************************************************
  */

#include "vl53l0x_i2c_platform.h"
#include "VL53L0X.h"

/* 只保存一个句柄指针。VL53L0X_Init() 通过 VL53L0X_I2C_Bind() 把它传进来，
 * 这样底层不写死 hi2c2，以后换 I2C1 或者换引脚只要改调用处即可。            */
static I2C_HandleTypeDef *s_hi2c = NULL;

/* 单次 I2C 传输超时。400kHz 下读写几十字节只要几百微秒，10ms 已经非常宽松。 */
#define VL53L0X_I2C_TIMEOUT_MS      10U

/* 供 VL53L0X_Init() 调用，绑定要用的 I2C 外设 */
void VL53L0X_I2C_Bind(I2C_HandleTypeDef *hi2c)
{
    s_hi2c = hi2c;
}

/* ---------------------------------------------------------------------------
 * 裸收发：以下 8 个函数对应官方 vl53l0x_i2c_platform.h 里的原型
 * 返回 0 表示成功，非 0 表示失败（官方约定，和 STATUS_OK 一致）
 * ------------------------------------------------------------------------- */

int32_t VL53L0X_write_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if ((s_hi2c == NULL) || (pdata == NULL) || (count <= 0)) {
        return -1;
    }

    /* 一次写多个字节：先发寄存器地址，再连发 count 个数据，中间不能有 STOP */
    if (HAL_I2C_Mem_Write(s_hi2c,
                          (uint16_t)address,
                          (uint16_t)index,
                          I2C_MEMADD_SIZE_8BIT,
                          pdata,
                          (uint16_t)count,
                          VL53L0X_I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }
    return 0;
}

int32_t VL53L0X_read_multi(uint8_t address, uint8_t index, uint8_t *pdata, int32_t count)
{
    if ((s_hi2c == NULL) || (pdata == NULL) || (count <= 0)) {
        return -1;
    }

    if (HAL_I2C_Mem_Read(s_hi2c,
                         (uint16_t)address,
                         (uint16_t)index,
                         I2C_MEMADD_SIZE_8BIT,
                         pdata,
                         (uint16_t)count,
                         VL53L0X_I2C_TIMEOUT_MS) != HAL_OK) {
        return -1;
    }
    return 0;
}

int32_t VL53L0X_write_byte(uint8_t address, uint8_t index, uint8_t data)
{
    uint8_t buffer = data;
    return VL53L0X_write_multi(address, index, &buffer, 1);
}

int32_t VL53L0X_write_word(uint8_t address, uint8_t index, uint16_t data)
{
    uint8_t buffer[BYTES_PER_WORD];

    /* 大端：高字节在前 */
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0x00FF);

    return VL53L0X_write_multi(address, index, buffer, BYTES_PER_WORD);
}

int32_t VL53L0X_write_dword(uint8_t address, uint8_t index, uint32_t data)
{
    uint8_t buffer[BYTES_PER_DWORD];

    /* 大端：从最高字节开始发 */
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)((data & 0x00FF0000U) >> 16);
    buffer[2] = (uint8_t)((data & 0x0000FF00U) >> 8);
    buffer[3] = (uint8_t)(data & 0x000000FFU);

    return VL53L0X_write_multi(address, index, buffer, BYTES_PER_DWORD);
}

int32_t VL53L0X_read_byte(uint8_t address, uint8_t index, uint8_t *pdata)
{
    return VL53L0X_read_multi(address, index, pdata, 1);
}

int32_t VL53L0X_read_word(uint8_t address, uint8_t index, uint16_t *pdata)
{
    uint8_t buffer[BYTES_PER_WORD];

    if (VL53L0X_read_multi(address, index, buffer, BYTES_PER_WORD) != 0) {
        return -1;
    }

    *pdata = ((uint16_t)buffer[0] << 8) | (uint16_t)buffer[1];
    return 0;
}

int32_t VL53L0X_read_dword(uint8_t address, uint8_t index, uint32_t *pdata)
{
    uint8_t buffer[BYTES_PER_DWORD];

    if (VL53L0X_read_multi(address, index, buffer, BYTES_PER_DWORD) != 0) {
        return -1;
    }

    *pdata = ((uint32_t)buffer[0] << 24) |
             ((uint32_t)buffer[1] << 16) |
             ((uint32_t)buffer[2] << 8)  |
             ((uint32_t)buffer[3]);

    return 0;
}

/**
  * @brief  关闭通信接口
  * @note   官方原型有这个东西（Windows 下是关串口）。单片机上没有需要关的东西，
  *         保留空实现只是为了和头文件里的声明配对，方便以后有代码调用。
  */
int32_t VL53L0X_comms_close(void)
{
    s_hi2c = NULL;
    return 0;
}
