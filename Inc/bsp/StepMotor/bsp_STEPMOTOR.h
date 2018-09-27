#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* 包含头文件 ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* 类型定义 ------------------------------------------------------------------*/
// 步进电机控制器类型定义
typedef struct{
  uint16_t  Pulse_Pin ; 	    // 定时器脉冲输出引脚
  uint32_t  Pulse_Channel;		// 定时器脉冲输出通道
  uint16_t  Ena_Pin ;         // 电机使能引脚编号
  uint16_t  Dir_Pin ;         // 电机方向引脚编号
  GPIO_TypeDef *Dir_Port;     // 电机方向引脚端口
  GPIO_TypeDef *Ena_Port;     // 电机使能引脚端口
}StepMotor_CtrlTypedef;

typedef struct {
  __IO uint8_t  X_Dir;      // X轴方向
  __IO uint8_t  Y_Dir;      // Y轴方向
  __IO uint32_t END_X;		 	// 终点坐标X
	__IO uint32_t END_Y;		 	// 终点坐标Y
	__IO uint32_t END_Pulse;	// 终点位置的脉冲数
	__IO uint32_t Active_Axis;	// 进给轴
	__IO int32_t  F_e;			 	// 函数方程
}InterPolation_Typedef;

/* 宏定义 --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM8                //定时器输出脉冲
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_UPDATA_IRQn             TIM8_UP_TIM13_IRQn
#define STEPMOTOR_TIM_UPDATA_IRQHandler       TIM8_UP_TIM13_IRQHandler

#define STEPMOTOR_PULSE_GPIO_CLK_ENABLE       __HAL_RCC_GPIOI_CLK_ENABLE
#define STEPMOTOR_TIM_PULSE_PORT              GPIOI               // TIM1脉冲输出引脚 4个通道对应的引脚在GPIOA 

/* X轴电机相关引脚定义 */
#define STEPMOTOR_TIM_CHANNEL1                TIM_CHANNEL_1			  // 定时器通道1//X轴
#define STEPMOTOR_TIM_PULSE_PIN_X             GPIO_PIN_5          // 输出脉冲给X轴电机控制器
#define GPIO_PIN_AF_TIMx                      GPIO_AF3_TIM8

#define STEPMOTOR_X_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_X_DIR_PORT                  GPIOC //WT.EDIT //GPIOD                            // 对应STEPMOTOR的DIR-（控制器使用共阳接法）
#define STEPMOTOR_X_DIR_PIN                   GPIO_PIN_1  //GPIO_PIN_3                      // 而DIR+直接接开发板的+5V(或3.3V)
#define GPIO_PIN_AF_AS_NORMAL                 GPIO_AF0_RTC_50Hz  

#define STEPMOTOR_X_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // 电机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_X_ENA_PORT                  GPIOC //WT.DEIT GPIOD                           // 对应STEPMOTOR的ENA-（控制器使用共阳接法）
#define STEPMOTOR_X_ENA_PIN                   GPIO_PIN_0 //GPIO_PIN_7                       // 而ENA+直接开发板的+5V(或3.3V) 

/* Y轴电机相关引脚定义 */
#define STEPMOTOR_TIM_CHANNEL2                TIM_CHANNEL_2			               // 定时器通道2//Y轴
#define STEPMOTOR_TIM_PULSE_PIN_Y             GPIO_PIN_6                       // 输出脉冲给Y轴电机控制器
#define GPIO_PIN_AF_TIMx                      GPIO_AF3_TIM8

#define STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // 电机旋转方向控制，如果悬空不接默认正转
#define STEPMOTOR_Y_DIR_PORT                  GPIOC // WT.EDIT //GPIOD                            // 对应STEPMOTOR的DIR-（控制器使用共阴接法）
#define STEPMOTOR_Y_DIR_PIN                   GPIO_PIN_3 //GPIO_PIN_11                      // 而DIR+直接接开发板的GND

#define STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // 电机使能控制，如果悬空不接默认使能电机
#define STEPMOTOR_Y_ENA_PORT                  GPIOC//WT.EDIT//GPIOF                            // 对应STEPMOTOR的ENA-（控制器使用共阴接法）
#define STEPMOTOR_Y_ENA_PIN                   GPIO_PIN_2//GPIO_PIN_11                       // 而ENA+直接开发板的GND 

#define STEPMOTOR_DIR_FORWARD(Axis)  \
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_RESET)//设置电机方向,参数Axis:当前活动轴
#define STEPMOTOR_DIR_REVERSAL(Axis) \
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT_ENABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_RESET)//设置电机使能,参数Axis:当前活动轴
#define STEPMOTOR_OUTPUT_DISABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_SET)//设置电机失能,参数Axis:当前活动轴

// 定义定时器预分频，定时器实际时钟频率为：168MHz/（STEPMOTOR_TIMx_PRESCALER+1）
#define STEPMOTOR_TIM_PRESCALER               15  // 步进电机驱动器细分设置为：   32  细分
//#define STEPMOTOR_TIM_PRESCALER               7  // 步进电机驱动器细分设置为：   16  细分
//#define STEPMOTOR_TIM_PRESCALER               15  // 步进电机驱动器细分设置为：   8  细分
//#define STEPMOTOR_TIM_PRESCALER               31  // 步进电机驱动器细分设置为：   4  细分
//#define STEPMOTOR_TIM_PRESCALER               63  // 步进电机驱动器细分设置为：   2  细分
//#define STEPMOTOR_TIM_PRESCALER               127  // 步进电机驱动器细分设置为：   1  细分

// 定义定时器周期，输出比较模式周期设置为0xFFFF
#define STEPMOTOR_TIM_PERIOD                  0xFFFF

#define AXIS_X                                0   // 各轴标号
#define AXIS_Y                                1
#define CCW                                   1   // 方向标记 逆时针
#define CW                                    0   // 方向标记 顺时针
/* 扩展变量 ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
/* 函数声明 ------------------------------------------------------------------*/
void STEPMOTOR_TIMx_Init( void );
void Llne_IncMove(uint32_t IncX,uint32_t IncY,uint32_t Speed);	// 直线插补功能函数
void Linear_Interpolation(int32_t coordsX,int32_t coordsY,int32_t Speed);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
