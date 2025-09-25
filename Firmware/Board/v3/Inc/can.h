/**
  ******************************************************************************
  * File Name          : CAN.h
  * Description        : This file provides code for the configuration
  *                      of the CAN instances.
  ******************************************************************************
  * This notice applies to any and all portions of this file
  * that are not between comment pairs USER CODE BEGIN and
  * USER CODE END. Other portions of this file, whether
  * inserted by the user or by software development tools
  * are owned by their respective copyright owners.
  *
  * Copyright (c) 2018 STMicroelectronics International N.V.
  * All rights reserved.
  *
  * Redistribution and use in source and binary forms, with or without
  * modification, are permitted, provided that the following conditions are met:
  *
  * 1. Redistribution of source code must retain the above copyright notice,
  *    this list of conditions and the following disclaimer.
  * 2. Redistributions in binary form must reproduce the above copyright notice,
  *    this list of conditions and the following disclaimer in the documentation
  *    and/or other materials provided with the distribution.
  * 3. Neither the name of STMicroelectronics nor the names of other
  *    contributors to this software may be used to endorse or promote products
  *    derived from this software without specific written permission.
  * 4. This software, including modifications and/or derivative works of this
  *    software, must execute solely and exclusively on microcontroller or
  *    microprocessor devices manufactured by or for STMicroelectronics.
  * 5. Redistribution and use of this software other than as permitted under
  *    this license is void and will automatically terminate your rights under
  *    this license.
  *
  * THIS SOFTWARE IS PROVIDED BY STMICROELECTRONICS AND CONTRIBUTORS "AS IS"
  * AND ANY EXPRESS, IMPLIED OR STATUTORY WARRANTIES, INCLUDING, BUT NOT
  * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
  * PARTICULAR PURPOSE AND NON-INFRINGEMENT OF THIRD PARTY INTELLECTUAL PROPERTY
  * RIGHTS ARE DISCLAIMED TO THE FULLEST EXTENT PERMITTED BY LAW. IN NO EVENT
  * SHALL STMICROELECTRONICS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
  * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
  * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
  * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
  * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
  * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
  *
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __can_H
#define __can_H
#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"
#include "main.h"

#define USE_CAN_MOTOR
// #define USE_CAN_BRUSH
// 驱动轮电机相关

#define CON_CAN_ID_TO_DRIVE            0X0001   //驱动轮电机驱动器ID
#define LEFT_DRIVE_CAN_ID              0x0002   //左驱动电机ID
#define RIGHT_DRIVE_CAN_ID             0x0003   //右驱动电机ID

// 滚刷电机相关
#define CON_CAN_ID_TO_BRUSH            0X0010   //滚刷电机驱动器ID
#define LEFT_BRUSH_CAN_ID              0x0012   //左滚刷电机ID
#define RIGHT_BRUSH_CAN_ID             0x0013   //右滚刷电机ID

// send cmd
#define MOTOR_HALL_CHANGE_AND_SPEED     0x09    //电机的霍尔变化 + 速度
#define MOTOR_VERSION                   0x0D    //暂时不做处理
#define MOTOR_LEFT_RIGHT_PHASE          0x0E    //暂时不做处理
#define MOTOR_LEFT_RIGHT_REVERSE        0x0F    //暂时不做处理
#define MOTOR_CONNECTION_STATE          0x10    //暂时不做处理
//send cmd end

// recve cmd
#define COM_CONFIRM                     0x00    //暂时不做处理
#define DRIVE_COMMAND0_SPEED            0X01    //轮子转速设置   第二个元素与第三个元素表示左电机速度（0-65535,32768为0速），四五元素表示右电机速度（0-65535,32768为0速）
#define DRIVE_COMMAND0_MODE             0X03    //驱动器运行模式（此驱动器有速度模式，位置模式，扭矩模式，电压模式；默认不允许改变使用速度模式）
#define DRIVE_COMMAND0_START_STOP       0X04    //驱动器状态控制 第二个元素是1表示启动（进入速度闭环模式），2表示停止（进入空闲模式）
#define DRIVE_COMMAND0_INQUIRY          0X05    //轮子在线问询   第二个元素是1表示z左电机 ，2表示右电机
#define DRIVE_COMMAND0_GET_SPEED        0X06    //轮子转速问询   第二个元素是1表示z左电机 ，2表示右电机
#define DRIVE_COMMAND0_PHASE_SEQUENCE   0X07    //设置电机相序   第二个元素是1表示z左电机 ，2表示右电机 （暂时不做这个功能）
#define DRIVE_COMMAND0_MOTOR_REVERSE    0X08    //设置电机转向   第二个元素是1表示z左电机 ，2表示右电机 第三个元素表示转向（0表示正转，1表示反转）
#define DRIVE_COMMAND0_MOTOR_EXCHANGE   0X09    //暂时不做处理
#define DRIVE_COMMAND0_AGEING_TEST      0X10    //暂时不做处理

#define DRIVE_COMMAND0_BATTERY_VOLATILE 0X30    //暂时不做处理

#define DRIVE_COMMAND0_GET_VERSION      0XF0    //暂时不做处理

#define DRIVE_COMMAND0_GET_SEQUENCE     0XE1    //暂时不做处理
#define DRIVE_COMMAND0_GET_REVVERSE     0XE2    //暂时不做处理
#define DRIVE_COMMAND0_GET_EXCHANGE     0XE3    //暂时不做处理

#define DRIVE_CLEAR_ERRORS                0xCD    //电机错误状态清除
#define DRIVE_MOTOR_CALIBRATION           0xCE    //电机校准
#define DRIVE_ENCODER_OFFSET_CALIBRATION  0xCF  //编码器校准
// recve cmd end

/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

extern CAN_HandleTypeDef hcan1;

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

extern void _Error_Handler(char *, int);

void MX_CAN1_Init(void);

/* USER CODE BEGIN Prototypes */

/* USER CODE END Prototypes */

#ifdef __cplusplus
}
#endif
#endif /*__ can_H */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
