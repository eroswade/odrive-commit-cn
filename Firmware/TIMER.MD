1. TIM14  1Ms时钟  触发定时时钟 TIM8_TRG_COM_TIM14_IRQHandler     ``` HAL_TIM_Base_Start_IT(&htim14); // TIM14定时时钟 ```
                    1. HAL_TIM_Base_Start_IT 在HAL_InitTick里被调用
                    2. HAL_InitTick 被  system_init调用
2. 在TIM8_TRG_COM_TIM14_IRQHandler里 触发TIM8 和TIM14中断
3. TIM8  24K 频率PWM    3次RepetitionCounter 进入一次中断  8K   |  TIM8_UP_TIM13_IRQHandler  疑问： T14是1K TIM8是8K。中断是怎么被调用的？
                    1. TIM8启动在 board.cpp/start_timers 里。中断也是在这里启动。 ```__HAL_TIM_ENABLE_IT(&htim8, TIM_IT_UPDATE);``` 
                    2. TIM14会 每1MS 强制同步一次TIM8。 
                    3. 在start_timers里还同步了TIM8 TIM1和TIM13的时钟  TIM1 比TIM8快1/4个周期（875个CYCLE  实际还要-64个CYCLE） TIM13比TIM0慢1/8周期
                    4. start_timers 在 start_adc_pwm 里被启动
4. TIM1和TIM8同步  TIM13也同步启动， 不过TIM13只给MEASURE_TIME用。 没有中断
5. TIM14 1KHZ  HAL库时基  守护TIM8
6. PWM 24K     [原解释](https://discourse.odriverobotics.com/t/increasing-the-pwm-frequency/2499)The PWM frequency = 8kHz * (TIM_1_8_RCR + 1). So 0 gives 8kHz, 2 gives 24kHz, 4 gives 40kHz, and 6 gives 56kHz.

## 一些官方说明 
1. https://discourse.odriverobotics.com/t/the-motor-timing-diagram/219
```
这个三角波是PWM模块中的计数器值，与第一张图中的三角波相同，只是我们有两个，每个电机一个。绘制的最左边的一个属于电动机M0，另一个属于M1。
我们希望在所有PWM信号都为低时对电流进行采样（此处将详细解释42），因此我们将ADC设置为在计数器达到0时触发。这被标记为图上的I0和I1点。当adc完成采样时，它们触发一个中断pwm_trig_ad_cb 21。这个中断然后唤醒电机控制线程，我们执行控制代码：这是一条歪歪扭扭的线，控制代码执行。
我们运行ADC并在顶部角落中断，标记为DC0和DC1，但这里ADC没有测量电流，我们用它来校准直流偏移。这不会唤醒电机控制线程，校准是通过中断中的过滤器完成的。
```
2. https://discourse.odriverobotics.com/t/odrive-for-pmsm/206/4?u=madcowswe 说明了测量位置保留了10%的PWM波时间， 用来做电流测量  2017年 联接了[TI的PDF](https://www.ti.com/lit/ug/tiducy7/tiducy7.pdf)