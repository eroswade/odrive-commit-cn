1. TIM14  1Ms时钟  触发定时时钟 TIM8_TRG_COM_TIM14_IRQHandler     ``` HAL_TIM_Base_Start_IT(&htim14); // TIM14定时时钟 ```
                    1. HAL_TIM_Base_Start_IT 在HAL_InitTick里被调用
                    2. HAL_InitTick 被  system_init调用
2. 在TIM8_TRG_COM_TIM14_IRQHandler里 触发TIM8 和TIM14中断
3. TIM8  24K 频率PWM    3次RepetitionCounter 进入一次中断  8K   |  TIM8_UP_TIM13_IRQHandler  疑问： T14是1K TIM8是8K。中断是怎么被调用的？
                    1. TIM8启动在 board.cpp/start_timers 里。 
                    2. TIM14会 每1MS 强制同步一次TIM8。 
                    3. 在start_timers里还同步了TIM8 TIM1和TIM13的时钟  TIM1 比TIM8快1/4个周期（875个CYCLE  实际还要-64个CYCLE） TIM13比TIM0慢1/8周期
                    4. start_timers 在 start_adc_pwm 里被启动
4. TIM1和TIM8同步  TIM13也同步启动， 不过TIM13只给MEASURE_TIME用。 没有中断
5. TIM14 1KHZ  HAL库时基  守护TIM8