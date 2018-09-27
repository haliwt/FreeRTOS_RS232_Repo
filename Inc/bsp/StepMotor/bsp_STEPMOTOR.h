#ifndef __STEPMOTOR_TIM_H__
#define __STEPMOTOR_TIM_H__

/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

/* ���Ͷ��� ------------------------------------------------------------------*/
// ����������������Ͷ���
typedef struct{
  uint16_t  Pulse_Pin ; 	    // ��ʱ�������������
  uint32_t  Pulse_Channel;		// ��ʱ���������ͨ��
  uint16_t  Ena_Pin ;         // ���ʹ�����ű��
  uint16_t  Dir_Pin ;         // ����������ű��
  GPIO_TypeDef *Dir_Port;     // ����������Ŷ˿�
  GPIO_TypeDef *Ena_Port;     // ���ʹ�����Ŷ˿�
}StepMotor_CtrlTypedef;

typedef struct {
  __IO uint8_t  X_Dir;      // X�᷽��
  __IO uint8_t  Y_Dir;      // Y�᷽��
  __IO uint32_t END_X;		 	// �յ�����X
	__IO uint32_t END_Y;		 	// �յ�����Y
	__IO uint32_t END_Pulse;	// �յ�λ�õ�������
	__IO uint32_t Active_Axis;	// ������
	__IO int32_t  F_e;			 	// ��������
}InterPolation_Typedef;

/* �궨�� --------------------------------------------------------------------*/
#define STEPMOTOR_TIMx                        TIM8                //��ʱ���������
#define STEPMOTOR_TIM_RCC_CLK_ENABLE()        __HAL_RCC_TIM8_CLK_ENABLE()
#define STEPMOTOR_TIM_RCC_CLK_DISABLE()       __HAL_RCC_TIM8_CLK_DISABLE()
#define STEPMOTOR_TIM_UPDATA_IRQn             TIM8_UP_TIM13_IRQn
#define STEPMOTOR_TIM_UPDATA_IRQHandler       TIM8_UP_TIM13_IRQHandler

#define STEPMOTOR_PULSE_GPIO_CLK_ENABLE       __HAL_RCC_GPIOI_CLK_ENABLE
#define STEPMOTOR_TIM_PULSE_PORT              GPIOI               // TIM1����������� 4��ͨ����Ӧ��������GPIOA 

/* X����������Ŷ��� */
#define STEPMOTOR_TIM_CHANNEL1                TIM_CHANNEL_1			  // ��ʱ��ͨ��1//X��
#define STEPMOTOR_TIM_PULSE_PIN_X             GPIO_PIN_5          // ��������X����������
#define GPIO_PIN_AF_TIMx                      GPIO_AF3_TIM8

#define STEPMOTOR_X_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_X_DIR_PORT                  GPIOC //WT.EDIT //GPIOD                            // ��ӦSTEPMOTOR��DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_X_DIR_PIN                   GPIO_PIN_1  //GPIO_PIN_3                      // ��DIR+ֱ�ӽӿ������+5V(��3.3V)
#define GPIO_PIN_AF_AS_NORMAL                 GPIO_AF0_RTC_50Hz  

#define STEPMOTOR_X_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // ���ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_X_ENA_PORT                  GPIOC //WT.DEIT GPIOD                           // ��ӦSTEPMOTOR��ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_X_ENA_PIN                   GPIO_PIN_0 //GPIO_PIN_7                       // ��ENA+ֱ�ӿ������+5V(��3.3V) 

/* Y����������Ŷ��� */
#define STEPMOTOR_TIM_CHANNEL2                TIM_CHANNEL_2			               // ��ʱ��ͨ��2//Y��
#define STEPMOTOR_TIM_PULSE_PIN_Y             GPIO_PIN_6                       // ��������Y����������
#define GPIO_PIN_AF_TIMx                      GPIO_AF3_TIM8

#define STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // �����ת������ƣ�������ղ���Ĭ����ת
#define STEPMOTOR_Y_DIR_PORT                  GPIOC // WT.EDIT //GPIOD                            // ��ӦSTEPMOTOR��DIR-��������ʹ�ù����ӷ���
#define STEPMOTOR_Y_DIR_PIN                   GPIO_PIN_3 //GPIO_PIN_11                      // ��DIR+ֱ�ӽӿ������GND

#define STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE()     __HAL_RCC_GPIOC_CLK_ENABLE()     // ���ʹ�ܿ��ƣ�������ղ���Ĭ��ʹ�ܵ��
#define STEPMOTOR_Y_ENA_PORT                  GPIOC//WT.EDIT//GPIOF                            // ��ӦSTEPMOTOR��ENA-��������ʹ�ù����ӷ���
#define STEPMOTOR_Y_ENA_PIN                   GPIO_PIN_2//GPIO_PIN_11                       // ��ENA+ֱ�ӿ������GND 

#define STEPMOTOR_DIR_FORWARD(Axis)  \
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_RESET)//���õ������,����Axis:��ǰ���
#define STEPMOTOR_DIR_REVERSAL(Axis) \
HAL_GPIO_WritePin(Stepmotor[Axis].Dir_Port,Stepmotor[Axis].Dir_Pin,GPIO_PIN_SET)

#define STEPMOTOR_OUTPUT_ENABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_RESET)//���õ��ʹ��,����Axis:��ǰ���
#define STEPMOTOR_OUTPUT_DISABLE(Axis)\
HAL_GPIO_WritePin(Stepmotor[Axis].Ena_Port,Stepmotor[Axis].Ena_Pin,GPIO_PIN_SET)//���õ��ʧ��,����Axis:��ǰ���

// ���嶨ʱ��Ԥ��Ƶ����ʱ��ʵ��ʱ��Ƶ��Ϊ��168MHz/��STEPMOTOR_TIMx_PRESCALER+1��
#define STEPMOTOR_TIM_PRESCALER               15  // �������������ϸ������Ϊ��   32  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               7  // �������������ϸ������Ϊ��   16  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               15  // �������������ϸ������Ϊ��   8  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               31  // �������������ϸ������Ϊ��   4  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               63  // �������������ϸ������Ϊ��   2  ϸ��
//#define STEPMOTOR_TIM_PRESCALER               127  // �������������ϸ������Ϊ��   1  ϸ��

// ���嶨ʱ�����ڣ�����Ƚ�ģʽ��������Ϊ0xFFFF
#define STEPMOTOR_TIM_PERIOD                  0xFFFF

#define AXIS_X                                0   // ������
#define AXIS_Y                                1
#define CCW                                   1   // ������ ��ʱ��
#define CW                                    0   // ������ ˳ʱ��
/* ��չ���� ------------------------------------------------------------------*/
extern TIM_HandleTypeDef htimx_STEPMOTOR;
/* �������� ------------------------------------------------------------------*/
void STEPMOTOR_TIMx_Init( void );
void Llne_IncMove(uint32_t IncX,uint32_t IncY,uint32_t Speed);	// ֱ�߲岹���ܺ���
void Linear_Interpolation(int32_t coordsX,int32_t coordsY,int32_t Speed);

#endif	/* __STEPMOTOR_TIM_H__ */
/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
