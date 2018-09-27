/**
  ******************************************************************************
  * 文件名程: bsp_STEPMOTOR.c 
  * 作    者: 硬石嵌入式开发团队
  * 版    本: V1.0
  * 编写日期: 2017-06-06
  * 功    能: 57&42步进电机基本旋转实现
  ******************************************************************************
  * 说明：
  * 本例程配套硬石stm32开发板YS-F1Pro使用。
  * 
  * 淘宝：
  * 论坛：http://www.ing10bbs.com
  * 版权归硬石嵌入式开发团队所有，请勿商用。
  ******************************************************************************
  */
/* 包含头文件 ----------------------------------------------------------------*/
#include "StepMotor/bsp_StepMotor.h"
#include <math.h>

/* 私有类型定义 --------------------------------------------------------------*/
/* 私有宏定义 ----------------------------------------------------------------*/
/* 私有变量 ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;

InterPolation_Typedef Line={0};	//直线插补参数值

  /* 需要修改直接在stepmotor.h头文件修改即可*/
  /* 步进电机控制引脚*/
const StepMotor_CtrlTypedef Stepmotor[2]={\
{STEPMOTOR_TIM_PULSE_PIN_X, STEPMOTOR_TIM_CHANNEL1,STEPMOTOR_X_ENA_PIN,STEPMOTOR_X_DIR_PIN,STEPMOTOR_X_DIR_PORT,STEPMOTOR_X_ENA_PORT},
{STEPMOTOR_TIM_PULSE_PIN_Y, STEPMOTOR_TIM_CHANNEL2,STEPMOTOR_Y_ENA_PIN,STEPMOTOR_Y_DIR_PIN,STEPMOTOR_Y_DIR_PORT,STEPMOTOR_Y_ENA_PORT},
};

__IO uint8_t  MotionStatus = 0;                   // 是否在运动？0：停止，1：运动
__IO int32_t  Step_Position[2] = {0} ;            // 当前位置  单位:脉冲数
__IO int32_t  Rel_Position[2] = {0} ;             // 当前位置  单位:脉冲数
__IO uint32_t Toggle_Pulse = 1000;
/* 扩展变量 ------------------------------------------------------------------*/
/* 私有函数原形 --------------------------------------------------------------*/
static void STEPMOTOR_GPIO_Init(void);
/* 函数体 --------------------------------------------------------------------*/
/**
  * 函数功能: STEPMOTOR相关GPIO初始化配置
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
static void STEPMOTOR_GPIO_Init()
{
	uint8_t i = 0;     
	GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* 电机方向控制引脚端口时钟使能 */
  STEPMOTOR_PULSE_GPIO_CLK_ENABLE();
  STEPMOTOR_X_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE();
  
	/* 电机使能控制引脚端口时钟使能 */
  STEPMOTOR_X_ENA_GPIO_CLK_ENABLE();
  STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE();
 
  for(i=0; i<2; i++)
  {
    /* 步进电机驱动器：脉冲输出 */
    GPIO_InitStruct.Pin = Stepmotor[i].Pulse_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_PIN_AF_TIMx;
    HAL_GPIO_Init(STEPMOTOR_TIM_PULSE_PORT, &GPIO_InitStruct);
    
    /* 步进电机驱动器：方向控制 */
    STEPMOTOR_DIR_FORWARD(i);   // 默认设置为顺时针方向
    GPIO_InitStruct.Pin = Stepmotor[i].Dir_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_PIN_AF_AS_NORMAL;
    HAL_GPIO_Init(Stepmotor[i].Dir_Port, &GPIO_InitStruct);
  
    /* 步进电机驱动器：使能控制 */
    STEPMOTOR_OUTPUT_ENABLE(i); // 默认使能
    GPIO_InitStruct.Pin = Stepmotor[i].Ena_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_PIN_AF_AS_NORMAL;
    HAL_GPIO_Init(Stepmotor[i].Ena_Port, &GPIO_InitStruct);
  }
}
/**
  * 函数功能: 步进电机驱动器定时器初始化
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 无
  */
void STEPMOTOR_TIMx_Init()
{
	uint8_t i = 0;	                           
	TIM_ClockConfigTypeDef sClockSourceConfig;  // 定时器时钟源配置
	TIM_OC_InitTypeDef sConfigOC;               // 定时器比较输出通道配置

	STEPMOTOR_TIM_RCC_CLK_ENABLE();
  
	/* STEPMOTOR相关GPIO初始化配置 */
	STEPMOTOR_GPIO_Init();
  
	/* 定时器基本环境配置 */
	htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                      	// 定时器编号
	htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;        	// 定时器预分频器
	htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;            // 计数方向：向上计数
	htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;          		  // 定时器周期
	htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;        // 时钟分频
	HAL_TIM_OnePulse_Init(&htimx_STEPMOTOR,TIM_OPMODE_SINGLE);
//  HAL_TIM_Base_Init(&htimx_STEPMOTOR);
	/* 定时器时钟源配置 */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       	// 使用内部时钟源
	HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

	/* 定时器比较输出配置 */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;                  // 比较输出模式：PWM模式2
  sConfigOC.Pulse = STEPMOTOR_TIM_PERIOD;              // 脉冲数
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          // 输出极性
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;        // 互补通道输出极性
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // 快速模式
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;         // 空闲电平
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;       // 互补通道空闲电平
	
	/* 使能比较输出通道 */
	for (i=0; i <2; i++)
	{
    HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, Stepmotor[i].Pulse_Channel);
		TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[i].Pulse_Channel, TIM_CCx_DISABLE);
	}
  
  	/* Enable the main output */
	__HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR);  
	/* 配置定时器中断优先级并使能 */
  HAL_NVIC_SetPriority(STEPMOTOR_TIM_UPDATA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(STEPMOTOR_TIM_UPDATA_IRQn);
  __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR,TIM_IT_UPDATE);
}
/**
	* 函数功能: 直线插补增量运动
	* 输入参数: X,Y:X,Y坐标增量,Speed:进给速度
	* 返 回 值: 无
	* 说		明: 函数实现直线插补功能,输入的坐标是坐标的增量,
	*						执行该函数之后,两个步进电机分别向X轴和Y轴步进IncX,IncY步
	*/
void Llne_IncMove(uint32_t IncX,uint32_t IncY,uint32_t Speed)
{
	Line.F_e = 0;							// 偏差方程置零
	Rel_Position[AXIS_X]= 0;	// 将当前位置视为原点
	Rel_Position[AXIS_Y]= 0;	
	
	Line.END_X = IncX;										 //终点坐标对应的脉冲数位置
	Line.END_Y = IncY;										 //终点坐标对应的脉冲数位置
	Line.END_Pulse = Line.END_Y + Line.END_X;
	Toggle_Pulse = Speed;
	Line.Active_Axis = AXIS_X;
	Line.F_e = Line.F_e - Line.END_Y;						//第一步进给X轴
	
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,Stepmotor[AXIS_X].Pulse_Channel,Toggle_Pulse);
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,Stepmotor[AXIS_Y].Pulse_Channel,Toggle_Pulse);
  __HAL_TIM_SET_AUTORELOAD(&htimx_STEPMOTOR,Toggle_Pulse*2);

	TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Line.Active_Axis].Pulse_Channel, TIM_CCx_ENABLE);
  HAL_TIM_Base_Start_IT(&htimx_STEPMOTOR);    // 使能定时器
  MotionStatus = 1;
}

/**
*函数功能: 任意直线插补
*输入参数: coordsX: X坐标增量,coordsY:Y坐标增量,Speed:插补速度(定时器计数值)
*返 回 值: 无
*说    明: 在当前位置以给定的坐标增量实现直线插补.
*/
void Linear_Interpolation( int32_t coordsX,int32_t coordsY, int32_t Speed)
{
  if(MotionStatus != 0) // 当前电机正在运转
    return ;
  /* 其他象限的直线跟第一象限是一样,只是电机运动方向不一样 */
  if(coordsX < 0)
  {
    Line.X_Dir = CCW;
    coordsX = -coordsX;
    STEPMOTOR_DIR_REVERSAL(AXIS_X);
  }
  else STEPMOTOR_DIR_FORWARD(AXIS_X);
  if(coordsY < 0)
  {
    Line.Y_Dir = CCW;
    coordsY = -coordsY;
    STEPMOTOR_DIR_REVERSAL(AXIS_Y);
  }
  else STEPMOTOR_DIR_FORWARD(AXIS_Y);
  Llne_IncMove(coordsX,coordsY,Speed);
}
/**
  * 函数功能: 定时器中断回调函数
  * 输入参数: 无
  * 返 回 值: 无
  * 说    明: 实现加减速过程
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t Axis = 0;
  Axis = Line.Active_Axis;   // 当前进给轴
  
  Step_Position[Axis]++;	// 记录绝对坐标位置
  Rel_Position[Axis]++;		// 记录增量运动坐标
		
  /* 根据上一次的偏差判断进给方向,同时计算下一次的偏差 */
  if(Line.F_e >= 0)
  {	
    Line.Active_Axis = AXIS_X;
    Line.F_e = Line.F_e-Line.END_Y;			// 第一象限的X轴进给时,偏差计算
    
    if(Axis != Line.Active_Axis)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Line.Active_Axis].Pulse_Channel, TIM_CCx_ENABLE); 
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Axis].Pulse_Channel, TIM_CCx_DISABLE);
    }
  }
  else 
  {
    Line.Active_Axis = AXIS_Y;
    Line.F_e = Line.F_e+Line.END_X;			// 第一象限的Y轴进给时,偏差计算
    if(Axis != Line.Active_Axis)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Line.Active_Axis].Pulse_Channel, TIM_CCx_ENABLE); 
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Axis].Pulse_Channel, TIM_CCx_DISABLE);
    }
  }
  /* 插补结束判断 */
  if( (Rel_Position[AXIS_X]+Rel_Position[AXIS_Y]) == Line.END_Pulse)
  {
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[AXIS_X].Pulse_Channel, TIM_CCx_DISABLE);
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[AXIS_Y].Pulse_Channel, TIM_CCx_DISABLE);    
    HAL_TIM_Base_Stop_IT(&htimx_STEPMOTOR);// 失能定时器
    MotionStatus = 0;
  }
  else 
  {
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,Stepmotor[Line.Active_Axis].Pulse_Channel,Toggle_Pulse);
    HAL_TIM_Base_Start(&htimx_STEPMOTOR);   // 使能定时器
  }
}

/******************* (C) COPYRIGHT 2015-2020 硬石嵌入式开发团队 *****END OF FILE****/
