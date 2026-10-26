#ifndef __OLED_H
#define __OLED_H

#include "main.h"

/* ===========================================================================
 * 0.96 寸 OLED 驱动（SSD1306 / 128x64 / 4 脚 I2C 模块）
 *
 * 本文件对应 hilab 主控工程 hh（CubeMX 工程），与参考工程 oled 的差别：
 *
 *   1) 主频：参考工程是 HSI 8MHz，本工程是 HSE 72MHz
 *      → 软件 I2C 的延时循环改成按 SystemCoreClock 自动换算，见 OLED.c
 *
 *   2) 引脚：本工程沿用 PB8/PB9，但这两根在 CubeMX 里被配成了
 *      GPIO_OUTPUT_PP（推挽）。软件 I2C 必须开漏，所以 OLED_Init() 会
 *      用 HAL_GPIO_Init() 把它们重新配成 GPIO_MODE_OUTPUT_OD 覆盖掉。
 *      （更干净的做法是在 CubeMX 里直接把 PB8/PB9 改成 Output Open Drain）
 *
 * 接线（与《管球平衡》主控板原理图一致）：
 *
 *     OLED1-1  GND
 *     OLED1-2  VCC  ---  +5V
 *     OLED1-3  SCL  ---  PB8
 *     OLED1-4  SDA  ---  PB9
 *
 * 模块自带上拉到 5V；PB8/PB9 在 STM32F103 上是 FT 引脚，可以承受到 5V。
 * 不占用硬件 I2C 外设（I2C2 留给 VL53L0X）。
 *
 * 坐标约定：
 *     Line：1 ~ 4，一行高 16 像素
 *     Col ：1 ~ 128，单位是像素（不是字符），一个 8x16 字符占 8 列
 * =========================================================================== */

/* 引脚定义：若以后改接线，只改这 4 个宏 */
#define OLED_SCL_GPIO_PORT      GPIOB
#define OLED_SCL_GPIO_PIN       GPIO_PIN_8
#define OLED_SDA_GPIO_PORT      GPIOB
#define OLED_SDA_GPIO_PIN       GPIO_PIN_9

/* SSD1306 从机地址：7 位地址 0x3C 左移一位后的 8 位写地址 */
#define OLED_I2C_ADDRESS        0x78

#define OLED_WIDTH              128
#define OLED_HEIGHT             64

/*
 * 用法：
 *     #include "OLED.h"
 *
 *     if (OLED_Init() == 0) { 错误处理 }        // 返回 0 表示 I2C 无应答
 *     OLED_ShowString(1, 1, "POS:");
 *     OLED_ShowFloat(1, 41, 6.35f, 2, 1);       // 显示 "6.3"
 *     OLED_ShowSignedNum(2, 1, -25, 3);
 *     OLED_Clear();
 */

/* 初始化 GPIO 与 SSD1306 并清屏。返回 1 成功，0 表示 I2C 上没收到应答 */
uint8_t OLED_Init(void);

/* 全屏清空 */
void OLED_Clear(void);

/* 在 (Line, Col) 处显示一个 8x16 字符，超出范围的字符按空格处理 */
void OLED_ShowChar(uint8_t Line, uint8_t Col, char Ch);

/* 显示以 '\0' 结尾的字符串，越出右边界自动截断 */
void OLED_ShowString(uint8_t Line, uint8_t Col, const char *Str);

/* 显示 Len 位无符号十进制数，不足 Len 位左补 0 */
void OLED_ShowNum(uint8_t Line, uint8_t Col, uint32_t Num, uint8_t Len);

/* 显示带符号十进制数：正数带 '+'，负数带 '-'，再跟 Len 位数字 */
void OLED_ShowSignedNum(uint8_t Line, uint8_t Col, int32_t Num, uint8_t Len);

/* 显示 Len 位十六进制数（大写），不足 Len 位左补 0 */
void OLED_ShowHexNum(uint8_t Line, uint8_t Col, uint32_t Num, uint8_t Len);

/* 显示浮点数：IntLen 位整数 + 小数点 + DecLen 位小数（不依赖 printf 浮点） */
void OLED_ShowFloat(uint8_t Line, uint8_t Col, float Val, uint8_t IntLen, uint8_t DecLen);

#endif /* __OLED_H */
