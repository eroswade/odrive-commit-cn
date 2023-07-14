/**
  ******************************************************************************
  * File Name          : TIM.c
  * Description        : This file provides code for the configuration
  *                      of the TIM instances.
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

  /* Includes ------------------------------------------------------------------*/
#include "tim.h"

#include "gpio.h"

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim13;

// APB1时钟：42MHz； APB1相连的定时器时钟：84MHz
// APB2时钟：84MHz； APB2相连的定时器时钟：168MHz
//TIM1、TIM8 ：168MHz
//其它 ：84MHz
//定时器  输入频率     周期                          功能
//TIM1    168MHz  168M /(3500 * 2)= 24KHz         # 1电机(M0) 的UVW驱动
//TIM2    84MHz   84M /(4096 * 2) = 10.25KHz      CH3(PB .10) 和CH4(PB .11) 作为pwm输出，duty相反
//TIM3    84MHz   用作计数                        1电机之旋编检测，计满到0xfffff
//TIM4    84MHz   用作计数                        # 2电机之旋编检测，计满到0xfffff
//TIM5    84MHz   用作计数                        定时器的CH3(PA .2) 和CH4(PA .3) 作为捕获输入口
//TIM8    168MHz  24KHz                           # 2电机(M1) 的UVW驱动
//TIM13   84MHz   8KHz                            启动时和TIM1及TIM8同步；任务耗时测量
//TIM14   84MHz   1KHz                            作为HAL库的时基



/* TIM1 init function */
void MX_TIM1_Init(void)
{
    TIM_ClockConfigTypeDef sClockSourceConfig;
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

    ///////////////////////////////////////////////////////////////////////
    // 以下配置外部时钟及分频

    //168MHz/(3500(TIM_1_8_PERIOD_CLOCKS)*2) = 24KHz  因为中心模式(TIM_COUNTERMODE_CENTERALIGNED3)： 0-3500 -0   7000个CYCLES
    // 24/（RepetitionCounter+1）=24/(2(TIM_1_8_RCR)+1) = 8K
    // 注意这里的PWM。 因为是中心对齐， 所以每次在点到CCR的数(也就是apply_pwm_timings设置的) 都会换一次向。
    // 所以PWM为设置的高平的时间是(3500-CCR)*2的cycles：
    //          因为0*CCR为低电平(TIM_OCMODE_PWM2) CCR-3500 - 3500-CCR为高电平 ， CCR-0为低电平。
    // 在0-3500-0的过程中， 中断计数被触发2次。 由于 TIM_1_8_RCR， 所以3次中断计数，会触发一次中断函数（TIM8的）
    // 所以WAVE FORM 和 timing_diagram_v3.png 显示的一样。
    //                  注意： __M__代表代码里的current_meas_cb   __C__代表dc_calib_cb     __A__代表pwm_update_cb
    htim1.Instance = TIM1;
    htim1.Init.Prescaler = 0;
    htim1.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;// 中心对齐 CMS寄存器
    htim1.Init.Period = TIM_1_8_PERIOD_CLOCKS;//3500  // Auto-Reload Register   ARR寄存器 重装
    htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    // TIM8控制的pwm波形为中间对齐，这里设置RCR=2，也就是每（2+1）次更新会中断一次。
    htim1.Init.RepetitionCounter = TIM_1_8_RCR;//2   多少次进一次中断 所以中断是8K， 由TIM8完成
    if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
    {
    _Error_Handler(__FILE__, __LINE__);
    }

    sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
    if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_OC_Init(&htim1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // CCR为捕获寄存器。 CNT为计数寄存器。 
    // 注意 CCR在  @apply_pwm_timings 里被设置  motor.cpp里
    sConfigOC.OCMode = TIM_OCMODE_PWM2;// PWM模式2， CNT>CCR为高电平。   模式为1的时候 CNT<CCR为高电平。
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;
    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_TIMING;
    if (HAL_TIM_OC_ConfigChannel(&htim1, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF;
    sBreakDeadTimeConfig.DeadTime = TIM_1_8_DEADTIME_CLOCKS;
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim1, &sBreakDeadTimeConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim1);

}


/* TIM2 init function */
void MX_TIM2_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;

    htim2.Instance = TIM2;
    htim2.Init.Prescaler = 0;
    htim2.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;
    htim2.Init.Period = TIM_APB1_PERIOD_CLOCKS;
    htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_PWM_Init(&htim2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM2;
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_LOW;
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.Pulse = TIM_APB1_PERIOD_CLOCKS + 1;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
    if (HAL_TIM_PWM_ConfigChannel(&htim2, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim2);

}
/* TIM3 init function */
void MX_TIM3_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim3.Instance = TIM3;
    htim3.Init.Prescaler = 0;
    htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim3.Init.Period = 0xffff;
    htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 4;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 4;
    if (HAL_TIM_Encoder_Init(&htim3, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}
/* TIM4 init function */
void MX_TIM4_Init(void)
{
    TIM_Encoder_InitTypeDef sConfig;
    TIM_MasterConfigTypeDef sMasterConfig;

    htim4.Instance = TIM4;
    htim4.Init.Prescaler = 0;
    htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim4.Init.Period = 0xffff;
    htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
    sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC1Filter = 4;
    sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
    sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
    sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
    sConfig.IC2Filter = 4;
    if (HAL_TIM_Encoder_Init(&htim4, &sConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}
/* TIM5 init function */
void MX_TIM5_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_IC_InitTypeDef sConfigIC;

    htim5.Instance = TIM5;
    htim5.Init.Prescaler = 0;
    htim5.Init.CounterMode = TIM_COUNTERMODE_UP;
    htim5.Init.Period = 0xFFFFFFFF;
    htim5.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
    if (HAL_TIM_IC_Init(&htim5) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim5, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_BOTHEDGE;
    sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
    sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
    sConfigIC.ICFilter = 15;
    if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_IC_ConfigChannel(&htim5, &sConfigIC, TIM_CHANNEL_4) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

}
/* TIM8 init function */
void MX_TIM8_Init(void)
{
    TIM_MasterConfigTypeDef sMasterConfig;
    TIM_OC_InitTypeDef sConfigOC;
    TIM_BreakDeadTimeConfigTypeDef sBreakDeadTimeConfig;

    // 在这里 PWM被设置为 168/（3500 *2）= 24K
    // 但上下都要计数。 24*2/3 = 16K 而且每3次产生一个中断16K TIM8中断  也就是62.5US
    htim8.Instance = TIM8;
    htim8.Init.Prescaler = 0;
    htim8.Init.CounterMode = TIM_COUNTERMODE_CENTERALIGNED3;// 向下和向上计数都产生比较中断，产生一个RCR计数
    htim8.Init.Period = TIM_1_8_PERIOD_CLOCKS;//一个PWM为3500CYCLES
    htim8.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;// 不分割
    htim8.Init.RepetitionCounter = TIM_1_8_RCR;// RCR 寄存器。 正常为0，  2代表每3次产生一次中断。  
    // 注意：这里的中断是计算的核心中断 ControlLoop_IRQHandler。  一次ControlLoop_IRQHandler占用两次中断
    // 也就是62.5us * 2 = 125 us => 修改一次PWM
    // ERPM / RPM = 极对数 
    if (HAL_TIM_PWM_Init(&htim8) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
    sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
    if (HAL_TIMEx_MasterConfigSynchronization(&htim8, &sMasterConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    sConfigOC.OCMode = TIM_OCMODE_PWM2; // PWM输出模式2  CNT<CCR  低电平
    sConfigOC.Pulse = 0;
    sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;// 输出极性 高电平
    sConfigOC.OCNPolarity = TIM_OCNPOLARITY_HIGH;// 互补输出通道极性 高
    sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;// 快速模式 禁用
    sConfigOC.OCIdleState = TIM_OCIDLESTATE_RESET;// 通道闲置状态  空闲非工作
    sConfigOC.OCNIdleState = TIM_OCNIDLESTATE_RESET;// 互补通道闲置状态 非工作
    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    if (HAL_TIM_PWM_ConfigChannel(&htim8, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    // 死区
    sBreakDeadTimeConfig.OffStateRunMode = TIM_OSSR_ENABLE;
    sBreakDeadTimeConfig.OffStateIDLEMode = TIM_OSSI_ENABLE;
    sBreakDeadTimeConfig.LockLevel = TIM_LOCKLEVEL_OFF; // 
    sBreakDeadTimeConfig.DeadTime = TIM_1_8_DEADTIME_CLOCKS; // 20个CYCLE的死区
    sBreakDeadTimeConfig.BreakState = TIM_BREAK_DISABLE;
    sBreakDeadTimeConfig.BreakPolarity = TIM_BREAKPOLARITY_HIGH;
    sBreakDeadTimeConfig.AutomaticOutput = TIM_AUTOMATICOUTPUT_DISABLE;
    if (HAL_TIMEx_ConfigBreakDeadTime(&htim8, &sBreakDeadTimeConfig) != HAL_OK)
    {
        _Error_Handler(__FILE__, __LINE__);
    }

    HAL_TIM_MspPostInit(&htim8);

}
/* TIM13 init function */
void MX_TIM13_Init(void)
{

  htim13.Instance = TIM13;
  htim13.Init.Prescaler = 0;
  htim13.Init.CounterMode = TIM_COUNTERMODE_UP;
  // TIM13重装时间和TIM1的下溢更新事件高度重合
  // TIM13的周期 2*3500*(2+1) * (84000000/168000000) – 1 = 3500*3 - 1
  // TIM1的“更新中断”周期正好也是“3500*3”（中间对齐，RCR=2） 那么低溢出周期就是：“3500*6”了
  // TIM1的输入频率是TIM13的2倍，TIM13的计数方式是“向上”（不是中间对齐）。所以说TIM13的reload频率和TIM1的低溢出“更新中断频率”是一致的，
  // 但是TIM13的初始计数值落后TIM1，落后了“TIM1_INIT_COUNT/2”也就是(3500/2-128)/2=811
  // TIM13另一个功能就是作为耗时计算的时间计时： TaskTimer::start()
  htim13.Init.Period = (2 * TIM_1_8_PERIOD_CLOCKS * (TIM_1_8_RCR+1)) * ((float)TIM_APB1_CLOCK_HZ / (float)TIM_1_8_CLOCK_HZ) - 1;
  htim13.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  if (HAL_TIM_Base_Init(&htim13) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* tim_baseHandle)
{

    if (tim_baseHandle->Instance == TIM1)
    {
        /* USER CODE BEGIN TIM1_MspInit 0 */

        /* USER CODE END TIM1_MspInit 0 */
          /* TIM1 clock enable */
        __HAL_RCC_TIM1_CLK_ENABLE();
        /* USER CODE BEGIN TIM1_MspInit 1 */

        /* USER CODE END TIM1_MspInit 1 */
    }
    else if (tim_baseHandle->Instance == TIM13)
    {
        /* USER CODE BEGIN TIM13_MspInit 0 */

        /* USER CODE END TIM13_MspInit 0 */
          /* TIM13 clock enable */
        __HAL_RCC_TIM13_CLK_ENABLE();
        /* USER CODE BEGIN TIM13_MspInit 1 */

        /* USER CODE END TIM13_MspInit 1 */
    }
}

void HAL_TIM_PWM_MspInit(TIM_HandleTypeDef* tim_pwmHandle)
{

    if (tim_pwmHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspInit 0 */

        /* USER CODE END TIM2_MspInit 0 */
          /* TIM2 clock enable */
        __HAL_RCC_TIM2_CLK_ENABLE();
        /* USER CODE BEGIN TIM2_MspInit 1 */

        /* USER CODE END TIM2_MspInit 1 */
    }
    else if (tim_pwmHandle->Instance == TIM8)
    {
        /* USER CODE BEGIN TIM8_MspInit 0 */

        /* USER CODE END TIM8_MspInit 0 */
          /* TIM8 clock enable */
        __HAL_RCC_TIM8_CLK_ENABLE();
        /* USER CODE BEGIN TIM8_MspInit 1 */

        /* USER CODE END TIM8_MspInit 1 */
    }
}

void HAL_TIM_Encoder_MspInit(TIM_HandleTypeDef* tim_encoderHandle)
{

    if (tim_encoderHandle->Instance == TIM3)
    {
        /* USER CODE BEGIN TIM3_MspInit 0 */

        /* USER CODE END TIM3_MspInit 0 */
          /* TIM3 clock enable */
        __HAL_RCC_TIM3_CLK_ENABLE();

        /* USER CODE BEGIN TIM3_MspInit 1 */

        /* USER CODE END TIM3_MspInit 1 */
    }
    else if (tim_encoderHandle->Instance == TIM4)
    {
        /* USER CODE BEGIN TIM4_MspInit 0 */

        /* USER CODE END TIM4_MspInit 0 */
          /* TIM4 clock enable */
        __HAL_RCC_TIM4_CLK_ENABLE();

        /* USER CODE BEGIN TIM4_MspInit 1 */

        /* USER CODE END TIM4_MspInit 1 */
    }
}

void HAL_TIM_IC_MspInit(TIM_HandleTypeDef* tim_icHandle)
{

    if (tim_icHandle->Instance == TIM5)
    {
        /* USER CODE BEGIN TIM5_MspInit 0 */

        /* USER CODE END TIM5_MspInit 0 */
          /* TIM5 clock enable */
        __HAL_RCC_TIM5_CLK_ENABLE();

        /* TIM5 interrupt Init */
        HAL_NVIC_SetPriority(TIM5_IRQn, 1, 0);// 使能TIM5中断   CH3(PA.2)和CH4(PA.3)作为捕获输入口
        HAL_NVIC_EnableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspInit 1 */

        /* USER CODE END TIM5_MspInit 1 */
    }
}
void HAL_TIM_MspPostInit(TIM_HandleTypeDef* timHandle)
{

    GPIO_InitTypeDef GPIO_InitStruct;
    if (timHandle->Instance == TIM1)
    {
        /* USER CODE BEGIN TIM1_MspPostInit 0 */

        /* USER CODE END TIM1_MspPostInit 0 */
          /**TIM1 GPIO Configuration
          PB13     ------> TIM1_CH1N
          PB14     ------> TIM1_CH2N
          PB15     ------> TIM1_CH3N
          PA8     ------> TIM1_CH1
          PA9     ------> TIM1_CH2
          PA10     ------> TIM1_CH3
          */
        GPIO_InitStruct.Pin = M0_AL_Pin | M0_BL_Pin | M0_CL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M0_AH_Pin | M0_BH_Pin | M0_CH_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM1;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM1_MspPostInit 1 */

        /* USER CODE END TIM1_MspPostInit 1 */
    }
    else if (timHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspPostInit 0 */

        /* USER CODE END TIM2_MspPostInit 0 */

          /**TIM2 GPIO Configuration
          PB10     ------> TIM2_CH3
          PB11     ------> TIM2_CH4
          */
        GPIO_InitStruct.Pin = AUX_L_Pin | AUX_H_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF1_TIM2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM2_MspPostInit 1 */

        /* USER CODE END TIM2_MspPostInit 1 */
    }
    else if (timHandle->Instance == TIM8)
    {
        /* USER CODE BEGIN TIM8_MspPostInit 0 */

        /* USER CODE END TIM8_MspPostInit 0 */

          /**TIM8 GPIO Configuration
          PA7     ------> TIM8_CH1N
          PB0     ------> TIM8_CH2N
          PB1     ------> TIM8_CH3N
          PC6     ------> TIM8_CH1
          PC7     ------> TIM8_CH2
          PC8     ------> TIM8_CH3
          */
        GPIO_InitStruct.Pin = M1_AL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(M1_AL_GPIO_Port, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M1_BL_Pin | M1_CL_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin = M1_AH_Pin | M1_BH_Pin | M1_CH_Pin;
        GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull = GPIO_NOPULL;
        GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
        GPIO_InitStruct.Alternate = GPIO_AF3_TIM8;
        HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

        /* USER CODE BEGIN TIM8_MspPostInit 1 */

        /* USER CODE END TIM8_MspPostInit 1 */
    }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* tim_baseHandle)
{

    if (tim_baseHandle->Instance == TIM1)
    {
        /* USER CODE BEGIN TIM1_MspDeInit 0 */

        /* USER CODE END TIM1_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM1_CLK_DISABLE();

        /* TIM1 interrupt Deinit */
      /* USER CODE BEGIN TIM1_MspDeInit 1 */

      /* USER CODE END TIM1_MspDeInit 1 */
    }
    else if (tim_baseHandle->Instance == TIM13)
    {
        /* USER CODE BEGIN TIM13_MspDeInit 0 */

        /* USER CODE END TIM13_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM13_CLK_DISABLE();

        /* TIM13 interrupt Deinit */
      /* USER CODE BEGIN TIM13:TIM8_UP_TIM13_IRQn disable */
        /**
        * Uncomment the line below to disable the "TIM8_UP_TIM13_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn); */
      /* USER CODE END TIM13:TIM8_UP_TIM13_IRQn disable */

      /* USER CODE BEGIN TIM13_MspDeInit 1 */

      /* USER CODE END TIM13_MspDeInit 1 */
    }
}

void HAL_TIM_PWM_MspDeInit(TIM_HandleTypeDef* tim_pwmHandle)
{

    if (tim_pwmHandle->Instance == TIM2)
    {
        /* USER CODE BEGIN TIM2_MspDeInit 0 */

        /* USER CODE END TIM2_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM2_CLK_DISABLE();
        /* USER CODE BEGIN TIM2_MspDeInit 1 */

        /* USER CODE END TIM2_MspDeInit 1 */
    }
    else if (tim_pwmHandle->Instance == TIM8)
    {
        /* USER CODE BEGIN TIM8_MspDeInit 0 */

        /* USER CODE END TIM8_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM8_CLK_DISABLE();

        /* TIM8 interrupt Deinit */
      /* USER CODE BEGIN TIM8:TIM8_UP_TIM13_IRQn disable */
        /**
        * Uncomment the line below to disable the "TIM8_UP_TIM13_IRQn" interrupt
        * Be aware, disabling shared interrupt may affect other IPs
        */
        /* HAL_NVIC_DisableIRQ(TIM8_UP_TIM13_IRQn); */
      /* USER CODE END TIM8:TIM8_UP_TIM13_IRQn disable */
      /* USER CODE BEGIN TIM8_MspDeInit 1 */

      /* USER CODE END TIM8_MspDeInit 1 */
    }
}

void HAL_TIM_Encoder_MspDeInit(TIM_HandleTypeDef* tim_encoderHandle)
{

    if (tim_encoderHandle->Instance == TIM3)
    {
        /* USER CODE BEGIN TIM3_MspDeInit 0 */

        /* USER CODE END TIM3_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM3_CLK_DISABLE();

        /**TIM3 GPIO Configuration
        PB4     ------> TIM3_CH1
        PB5     ------> TIM3_CH2
        */
        HAL_GPIO_DeInit(GPIOB, M0_ENC_A_Pin | M0_ENC_B_Pin);

        /* USER CODE BEGIN TIM3_MspDeInit 1 */

        /* USER CODE END TIM3_MspDeInit 1 */
    }
    else if (tim_encoderHandle->Instance == TIM4)
    {
        /* USER CODE BEGIN TIM4_MspDeInit 0 */

        /* USER CODE END TIM4_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM4_CLK_DISABLE();

        /**TIM4 GPIO Configuration
        PB6     ------> TIM4_CH1
        PB7     ------> TIM4_CH2
        */
        HAL_GPIO_DeInit(GPIOB, M1_ENC_A_Pin | M1_ENC_B_Pin);

        /* USER CODE BEGIN TIM4_MspDeInit 1 */

        /* USER CODE END TIM4_MspDeInit 1 */
    }
}

void HAL_TIM_IC_MspDeInit(TIM_HandleTypeDef* tim_icHandle)
{

    if (tim_icHandle->Instance == TIM5)
    {
        /* USER CODE BEGIN TIM5_MspDeInit 0 */

        /* USER CODE END TIM5_MspDeInit 0 */
          /* Peripheral clock disable */
        __HAL_RCC_TIM5_CLK_DISABLE();

        /**TIM5 GPIO Configuration
        PA2     ------> TIM5_CH3
        PA3     ------> TIM5_CH4
        */
        HAL_GPIO_DeInit(GPIOA, GPIO_3_Pin | GPIO_4_Pin);

        /* TIM5 interrupt Deinit */
        HAL_NVIC_DisableIRQ(TIM5_IRQn);
        /* USER CODE BEGIN TIM5_MspDeInit 1 */

        /* USER CODE END TIM5_MspDeInit 1 */
    }
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/**
  * @}
  */

  /**
    * @}
    */

    /************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/
