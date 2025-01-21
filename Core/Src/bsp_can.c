/******************************************************************************
/// @brief
/// @copyright Copyright (c) 2017 <dji-innovations, Corp. RM Dept.>
/// @license MIT License
/// Permission is hereby granted, free of charge, to any person obtaining a copy
/// of this software and associated documentation files (the "Software"), to deal
/// in the Software without restriction,including without limitation the rights
/// to use, copy, modify, merge, publish, distribute, sublicense,and/or sell
/// copies of the Software, and to permit persons to whom the Software is furnished
/// to do so,subject to the following conditions:
///
/// The above copyright notice and this permission notice shall be included in
/// all copies or substantial portions of the Software.
///
/// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
/// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
/// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
/// THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
/// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
/// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
/// THE SOFTWARE.
*******************************************************************************/

#include "bsp_can.h"
#include "gpio.h"
#include "log.h"

#include "can.h"
// #include "cmsis_os.h"

// moto_measure_t moto_pit;
// moto_measure_t moto_yaw;
// moto_measure_t moto_poke;	//拨单电机
moto_measure_t moto_chassis[4] = {0};  // 4 chassis moto
moto_measure_t moto_info;

void get_total_angle(moto_measure_t* p);

/*******************************************************************************************
 * @Func		my_can_filter_init
 * @Brief    CAN1和CAN2滤波器配置
 * @Param		CAN_HandleTypeDef* hcan
 * @Retval		None
 * @Date     2015/11/30
 *******************************************************************************************/

void my_can_filter_init_recv_all(CAN_HandleTypeDef* _hcan) {
  // CAN1 & CAN2 use the same filter configuration
  CAN_FilterTypeDef CAN_FilterConfigStructure;

  // 配置 CAN1 和 CAN2 使用的过滤器
  CAN_FilterConfigStructure.FilterBank = 0;
  CAN_FilterConfigStructure.FilterMode = CAN_FILTERMODE_IDMASK;       // 过滤器模式
  CAN_FilterConfigStructure.FilterScale = CAN_FILTERSCALE_32BIT;      // 32位过滤器
  CAN_FilterConfigStructure.FilterIdHigh = 0x0000;                    // 过滤器ID高位
  CAN_FilterConfigStructure.FilterIdLow = 0x0000;                     // 过滤器ID低位
  CAN_FilterConfigStructure.FilterMaskIdHigh = 0x0000;                // 过滤器掩码高位
  CAN_FilterConfigStructure.FilterMaskIdLow = 0x0000;                 // 过滤器掩码低位
  CAN_FilterConfigStructure.FilterFIFOAssignment = CAN_FILTER_FIFO0;  // 指定FIFO
  CAN_FilterConfigStructure.FilterActivation = ENABLE;                // 启用过滤器
                                                                      // CAN1的第一个过滤器（0号过滤器）
  // 配置 CAN1 过滤器
  if (HAL_CAN_ConfigFilter(_hcan, &CAN_FilterConfigStructure) != HAL_OK) {
    // 错误处理代码
  }
}

/*******************************************************************************************
 * @Func			void HAL_CAN_RxCpltCallback(CAN_HandleTypeDef* _hcan)
 * @Brief    这是一个回调函数,都不用声明
 * @Param
 * @Retval		None
 * @Date     2015/11/24
 *******************************************************************************************/
void RxFifo1MsgPendingCallback(CAN_HandleTypeDef* _hcan) {
  CAN_RxHeaderTypeDef RxHeader;
  uint8_t RxData[8];  // 假设最多 8 字节数据

  // 获取接收到的 CAN 消息
  if (HAL_CAN_GetRxMessage(_hcan, CAN_RX_FIFO0, &RxHeader, RxData) == HAL_OK) {
    switch (RxHeader.StdId) {
      case CAN_3510Moto1_ID:
      case CAN_3510Moto2_ID:
      case CAN_3510Moto3_ID:
      case CAN_3510Moto4_ID: {
        static uint8_t i;
        i = RxHeader.StdId - CAN_3510Moto1_ID;

        if (moto_chassis[i].msg_cnt++ <= 50) {
          moto_chassis[i].angle = (uint16_t)(RxData[0] << 8 | RxData[1]);
          moto_chassis[i].offset_angle = moto_chassis[i].angle;
        } else {
          get_moto_measure(&moto_chassis[i], RxData);
        }
        get_moto_measure(&moto_info, RxData);
      } break;
    }
  }

  // 重新启用 CAN 接收中断
  __HAL_CAN_ENABLE_IT(_hcan, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/*******************************************************************************************
 * @Func			void get_moto_measure(moto_measure_t *ptr, CAN_HandleTypeDef* hcan)
 * @Brief    接收云台电机,3510电机通过CAN发过来的信息
 * @Param
 * @Retval		None
 * @Date     2015/11/24
 *******************************************************************************************/
void get_moto_measure(moto_measure_t* ptr, uint8_t* RxData) {
  ptr->last_angle = ptr->angle;
  ptr->angle = (uint16_t)(RxData[0] << 8 | RxData[1]);
  ptr->real_current = (int16_t)(RxData[2] << 8 | RxData[3]);
  ptr->speed_rpm = ptr->real_current;
  ptr->given_current = (int16_t)(RxData[4] << 8 | RxData[5]) / -5;
  ptr->hall = RxData[6];

  if (ptr->angle - ptr->last_angle > 4096)
    ptr->round_cnt--;
  else if (ptr->angle - ptr->last_angle < -4096)
    ptr->round_cnt++;

  ptr->total_angle = ptr->round_cnt * 8192 + ptr->angle - ptr->offset_angle;
}

#define ABS(x) ((x > 0) ? (x) : (-x))
/**
 *@bref 电机上电角度=0， 之后用这个函数更新3510电机的相对开机后（为0）的相对角度。
 */
void get_total_angle(moto_measure_t* p) {
  int res1, res2, delta;
  if (p->angle < p->last_angle) {            // 可能的情况
    res1 = p->angle + 8192 - p->last_angle;  // 正转，delta=+
    res2 = p->angle - p->last_angle;         // 反转	delta=-
  } else {                                   // angle > last
    res1 = p->angle - 8192 - p->last_angle;  // 反转	delta -
    res2 = p->angle - p->last_angle;         // 正转	delta +
  }
  // 不管正反转，肯定是转的角度小的那个是真的
  if (ABS(res1) < ABS(res2))
    delta = res1;
  else
    delta = res2;

  p->total_angle += delta;
  p->last_angle = p->angle;
}

void set_moto_current(CAN_HandleTypeDef* hcan, int16_t iq1, int16_t iq2, int16_t iq3, int16_t iq4) {
  CAN_TxHeaderTypeDef TxHeader;
  uint32_t txMailbox = 0;
  uint8_t TxData[8];

  // 设置消息头
  TxHeader.StdId = 0x200;
  TxHeader.IDE = CAN_ID_STD;
  TxHeader.RTR = CAN_RTR_DATA;
  TxHeader.DLC = 0x08;

  // 设置数据字段
  TxData[0] = iq1 >> 8;
  TxData[1] = iq1;
  TxData[2] = iq2 >> 8;
  TxData[3] = iq2;
  TxData[4] = iq3 >> 8;
  TxData[5] = iq3;
  TxData[6] = iq4 >> 8;
  TxData[7] = iq4;

  // 发送 CAN 消息
  if (HAL_CAN_AddTxMessage(hcan, &TxHeader, TxData, &txMailbox) != HAL_OK) {
    HAL_GPIO_WritePin(CAN_TX_ERR_GPIO_Port, CAN_TX_ERR_Pin, GPIO_PIN_RESET);
    LOGE("CAN Send Error");
  }
}
