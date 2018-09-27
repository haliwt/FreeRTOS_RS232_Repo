/**
  ******************************************************************************
  * �ļ�����: bsp_STEPMOTOR.c 
  * ��    ��: ӲʯǶ��ʽ�����Ŷ�
  * ��    ��: V1.0
  * ��д����: 2017-06-06
  * ��    ��: 57&42�������������תʵ��
  ******************************************************************************
  * ˵����
  * ����������Ӳʯstm32������YS-F1Proʹ�á�
  * 
  * �Ա���
  * ��̳��http://www.ing10bbs.com
  * ��Ȩ��ӲʯǶ��ʽ�����Ŷ����У��������á�
  ******************************************************************************
  */
/* ����ͷ�ļ� ----------------------------------------------------------------*/
#include "StepMotor/bsp_StepMotor.h"
#include <math.h>

/* ˽�����Ͷ��� --------------------------------------------------------------*/
/* ˽�к궨�� ----------------------------------------------------------------*/
/* ˽�б��� ------------------------------------------------------------------*/
TIM_HandleTypeDef htimx_STEPMOTOR;

InterPolation_Typedef Line={0};	//ֱ�߲岹����ֵ

  /* ��Ҫ�޸�ֱ����stepmotor.hͷ�ļ��޸ļ���*/
  /* ���������������*/
const StepMotor_CtrlTypedef Stepmotor[2]={\
{STEPMOTOR_TIM_PULSE_PIN_X, STEPMOTOR_TIM_CHANNEL1,STEPMOTOR_X_ENA_PIN,STEPMOTOR_X_DIR_PIN,STEPMOTOR_X_DIR_PORT,STEPMOTOR_X_ENA_PORT},
{STEPMOTOR_TIM_PULSE_PIN_Y, STEPMOTOR_TIM_CHANNEL2,STEPMOTOR_Y_ENA_PIN,STEPMOTOR_Y_DIR_PIN,STEPMOTOR_Y_DIR_PORT,STEPMOTOR_Y_ENA_PORT},
};

__IO uint8_t  MotionStatus = 0;                   // �Ƿ����˶���0��ֹͣ��1���˶�
__IO int32_t  Step_Position[2] = {0} ;            // ��ǰλ��  ��λ:������
__IO int32_t  Rel_Position[2] = {0} ;             // ��ǰλ��  ��λ:������
__IO uint32_t Toggle_Pulse = 1000;
/* ��չ���� ------------------------------------------------------------------*/
/* ˽�к���ԭ�� --------------------------------------------------------------*/
static void STEPMOTOR_GPIO_Init(void);
/* ������ --------------------------------------------------------------------*/
/**
  * ��������: STEPMOTOR���GPIO��ʼ������
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
static void STEPMOTOR_GPIO_Init()
{
	uint8_t i = 0;     
	GPIO_InitTypeDef GPIO_InitStruct; 
  
  /* �������������Ŷ˿�ʱ��ʹ�� */
  STEPMOTOR_PULSE_GPIO_CLK_ENABLE();
  STEPMOTOR_X_DIR_GPIO_CLK_ENABLE();
  STEPMOTOR_Y_DIR_GPIO_CLK_ENABLE();
  
	/* ���ʹ�ܿ������Ŷ˿�ʱ��ʹ�� */
  STEPMOTOR_X_ENA_GPIO_CLK_ENABLE();
  STEPMOTOR_Y_ENA_GPIO_CLK_ENABLE();
 
  for(i=0; i<2; i++)
  {
    /* ���������������������� */
    GPIO_InitStruct.Pin = Stepmotor[i].Pulse_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_PIN_AF_TIMx;
    HAL_GPIO_Init(STEPMOTOR_TIM_PULSE_PORT, &GPIO_InitStruct);
    
    /* ���������������������� */
    STEPMOTOR_DIR_FORWARD(i);   // Ĭ������Ϊ˳ʱ�뷽��
    GPIO_InitStruct.Pin = Stepmotor[i].Dir_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_PIN_AF_AS_NORMAL;
    HAL_GPIO_Init(Stepmotor[i].Dir_Port, &GPIO_InitStruct);
  
    /* ���������������ʹ�ܿ��� */
    STEPMOTOR_OUTPUT_ENABLE(i); // Ĭ��ʹ��
    GPIO_InitStruct.Pin = Stepmotor[i].Ena_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    GPIO_InitStruct.Alternate = GPIO_PIN_AF_AS_NORMAL;
    HAL_GPIO_Init(Stepmotor[i].Ena_Port, &GPIO_InitStruct);
  }
}
/**
  * ��������: ���������������ʱ����ʼ��
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ��
  */
void STEPMOTOR_TIMx_Init()
{
	uint8_t i = 0;	                           
	TIM_ClockConfigTypeDef sClockSourceConfig;  // ��ʱ��ʱ��Դ����
	TIM_OC_InitTypeDef sConfigOC;               // ��ʱ���Ƚ����ͨ������

	STEPMOTOR_TIM_RCC_CLK_ENABLE();
  
	/* STEPMOTOR���GPIO��ʼ������ */
	STEPMOTOR_GPIO_Init();
  
	/* ��ʱ�������������� */
	htimx_STEPMOTOR.Instance = STEPMOTOR_TIMx;                      	// ��ʱ�����
	htimx_STEPMOTOR.Init.Prescaler = STEPMOTOR_TIM_PRESCALER;        	// ��ʱ��Ԥ��Ƶ��
	htimx_STEPMOTOR.Init.CounterMode = TIM_COUNTERMODE_UP;            // �����������ϼ���
	htimx_STEPMOTOR.Init.Period = STEPMOTOR_TIM_PERIOD;          		  // ��ʱ������
	htimx_STEPMOTOR.Init.ClockDivision=TIM_CLOCKDIVISION_DIV1;        // ʱ�ӷ�Ƶ
	HAL_TIM_OnePulse_Init(&htimx_STEPMOTOR,TIM_OPMODE_SINGLE);
//  HAL_TIM_Base_Init(&htimx_STEPMOTOR);
	/* ��ʱ��ʱ��Դ���� */
	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;       	// ʹ���ڲ�ʱ��Դ
	HAL_TIM_ConfigClockSource(&htimx_STEPMOTOR, &sClockSourceConfig);

	/* ��ʱ���Ƚ�������� */
	sConfigOC.OCMode = TIM_OCMODE_PWM2;                  // �Ƚ����ģʽ��PWMģʽ2
  sConfigOC.Pulse = STEPMOTOR_TIM_PERIOD;              // ������
	sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;          // �������
  sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;        // ����ͨ���������
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;           // ����ģʽ
  sConfigOC.OCIdleState = TIM_OCIDLESTATE_SET;         // ���е�ƽ
  sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_SET;       // ����ͨ�����е�ƽ
	
	/* ʹ�ܱȽ����ͨ�� */
	for (i=0; i <2; i++)
	{
    HAL_TIM_OC_ConfigChannel(&htimx_STEPMOTOR, &sConfigOC, Stepmotor[i].Pulse_Channel);
		TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[i].Pulse_Channel, TIM_CCx_DISABLE);
	}
  
  	/* Enable the main output */
	__HAL_TIM_MOE_ENABLE(&htimx_STEPMOTOR);  
	/* ���ö�ʱ���ж����ȼ���ʹ�� */
  HAL_NVIC_SetPriority(STEPMOTOR_TIM_UPDATA_IRQn, 0, 0);
	HAL_NVIC_EnableIRQ(STEPMOTOR_TIM_UPDATA_IRQn);
  __HAL_TIM_CLEAR_IT(&htimx_STEPMOTOR,TIM_IT_UPDATE);
}
/**
	* ��������: ֱ�߲岹�����˶�
	* �������: X,Y:X,Y��������,Speed:�����ٶ�
	* �� �� ֵ: ��
	* ˵		��: ����ʵ��ֱ�߲岹����,��������������������,
	*						ִ�иú���֮��,������������ֱ���X���Y�Ჽ��IncX,IncY��
	*/
void Llne_IncMove(uint32_t IncX,uint32_t IncY,uint32_t Speed)
{
	Line.F_e = 0;							// ƫ�������
	Rel_Position[AXIS_X]= 0;	// ����ǰλ����Ϊԭ��
	Rel_Position[AXIS_Y]= 0;	
	
	Line.END_X = IncX;										 //�յ������Ӧ��������λ��
	Line.END_Y = IncY;										 //�յ������Ӧ��������λ��
	Line.END_Pulse = Line.END_Y + Line.END_X;
	Toggle_Pulse = Speed;
	Line.Active_Axis = AXIS_X;
	Line.F_e = Line.F_e - Line.END_Y;						//��һ������X��
	
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,Stepmotor[AXIS_X].Pulse_Channel,Toggle_Pulse);
  __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,Stepmotor[AXIS_Y].Pulse_Channel,Toggle_Pulse);
  __HAL_TIM_SET_AUTORELOAD(&htimx_STEPMOTOR,Toggle_Pulse*2);

	TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Line.Active_Axis].Pulse_Channel, TIM_CCx_ENABLE);
  HAL_TIM_Base_Start_IT(&htimx_STEPMOTOR);    // ʹ�ܶ�ʱ��
  MotionStatus = 1;
}

/**
*��������: ����ֱ�߲岹
*�������: coordsX: X��������,coordsY:Y��������,Speed:�岹�ٶ�(��ʱ������ֵ)
*�� �� ֵ: ��
*˵    ��: �ڵ�ǰλ���Ը�������������ʵ��ֱ�߲岹.
*/
void Linear_Interpolation( int32_t coordsX,int32_t coordsY, int32_t Speed)
{
  if(MotionStatus != 0) // ��ǰ���������ת
    return ;
  /* �������޵�ֱ�߸���һ������һ��,ֻ�ǵ���˶�����һ�� */
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
  * ��������: ��ʱ���жϻص�����
  * �������: ��
  * �� �� ֵ: ��
  * ˵    ��: ʵ�ּӼ��ٹ���
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  uint32_t Axis = 0;
  Axis = Line.Active_Axis;   // ��ǰ������
  
  Step_Position[Axis]++;	// ��¼��������λ��
  Rel_Position[Axis]++;		// ��¼�����˶�����
		
  /* ������һ�ε�ƫ���жϽ�������,ͬʱ������һ�ε�ƫ�� */
  if(Line.F_e >= 0)
  {	
    Line.Active_Axis = AXIS_X;
    Line.F_e = Line.F_e-Line.END_Y;			// ��һ���޵�X�����ʱ,ƫ�����
    
    if(Axis != Line.Active_Axis)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Line.Active_Axis].Pulse_Channel, TIM_CCx_ENABLE); 
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Axis].Pulse_Channel, TIM_CCx_DISABLE);
    }
  }
  else 
  {
    Line.Active_Axis = AXIS_Y;
    Line.F_e = Line.F_e+Line.END_X;			// ��һ���޵�Y�����ʱ,ƫ�����
    if(Axis != Line.Active_Axis)
    {
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Line.Active_Axis].Pulse_Channel, TIM_CCx_ENABLE); 
      TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[Axis].Pulse_Channel, TIM_CCx_DISABLE);
    }
  }
  /* �岹�����ж� */
  if( (Rel_Position[AXIS_X]+Rel_Position[AXIS_Y]) == Line.END_Pulse)
  {
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[AXIS_X].Pulse_Channel, TIM_CCx_DISABLE);
    TIM_CCxChannelCmd(STEPMOTOR_TIMx, Stepmotor[AXIS_Y].Pulse_Channel, TIM_CCx_DISABLE);    
    HAL_TIM_Base_Stop_IT(&htimx_STEPMOTOR);// ʧ�ܶ�ʱ��
    MotionStatus = 0;
  }
  else 
  {
    __HAL_TIM_SET_COMPARE(&htimx_STEPMOTOR,Stepmotor[Line.Active_Axis].Pulse_Channel,Toggle_Pulse);
    HAL_TIM_Base_Start(&htimx_STEPMOTOR);   // ʹ�ܶ�ʱ��
  }
}

/******************* (C) COPYRIGHT 2015-2020 ӲʯǶ��ʽ�����Ŷ� *****END OF FILE****/
