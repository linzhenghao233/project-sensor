/*
 * @Author: i want to 舞动乾坤
 * @Date: 2024-07-23 18:30:26
 * @LastEditors: i want to 舞动乾坤
 * @LastEditTime: 2024-07-23 18:53:13
 * @FilePath: \pwm_sg90\main\sg90.c
 * @Description: sg90舵机驱动代码
 * 
 * Copyright (c) 2024 by i want to 舞动乾坤, All Rights Reserved. 
 */

#include <freeRtos/FreeRTOS.h>
#include <driver/ledc.h>
#include <driver/gpio.h>
#include <freeRtos/task.h>
#define sg90_pin GPIO_NUM_5//信号线的引脚
#define sg90_freq_hz (50)//sg90频率
#define sg90_timer0 LEDC_TIMER_0//定时器0
#define sg90_timer_channel LEDC_CHANNEL_0 //通道0
#define sg90_speedMode LEDC_LOW_SPEED_MODE//低速模式
//为了符合 0.5ms~2.5ms的脉宽  将500~2500 映射到 0度~180度 
#define sg90_min_pulse_width_us 500 //映射到500 的最小脉冲宽度 
#define sg90_max_pulse_width_us 2500//映射到2500的最大脉冲宽度 

/**
 * @description: sg90初始化
 * @param 无
 * @return {*}
 */
void sg90_init()
{
    ledc_timer_config_t ledc_timer_InitStructure=
    {
        .clk_cfg = LEDC_AUTO_CLK,     //自动选择时钟
        .duty_resolution=LEDC_TIMER_13_BIT,//13bit位分辨率
        .freq_hz=sg90_freq_hz,//50hz     T=1/50 =0.02s =20ms 周期
        .speed_mode=sg90_speedMode,//模式位低速模式
        .timer_num=sg90_timer0//使用定时器0
    };
    ledc_timer_config(&ledc_timer_InitStructure);


    ledc_channel_config_t ledc_channel_InitStructure=
    {
        .channel=sg90_timer_channel,//通道0
        .duty=0,//占空比 后面可以再设置 范围：0~ 2^13 -1
        .gpio_num=sg90_pin,
        .hpoint=0,
        .intr_type=LEDC_INTR_DISABLE,//中断使能关闭
        .speed_mode=sg90_speedMode,//低速模式
        .timer_sel=sg90_timer0//定时器 0
    };

    ledc_channel_config(&ledc_channel_InitStructure);
}


/**
 * @description: 设置sg90占空比
 * @param: duty 设置的占空比 范围：0~(2^13) -1
 * @return {*}
 */
void sg90_SetDuty(uint32_t duty)
{
    ledc_set_duty(sg90_speedMode,sg90_timer_channel,duty);
    ledc_update_duty(sg90_speedMode,sg90_timer_channel);
}


/**
 * @description: 设置sg90转动的角度
 * @param: angle 角度 范围：0~180
 * @return {*}
 */
void sg90_SetAngle(float angle)
{
    uint16_t pulse_width_us=0;
    uint32_t duty;
    if(angle>180){angle=180;}
    else if(angle<0){angle=0;}
    //计算过程：0度 对应 500us  180度 对应 2500us  即线性关系 y=kx+b  ，代入 （x=0,y=500）以及(x=180,y=2500)  求解k 和 b ，由传来的参数angle得到最终映射的带宽us数
    pulse_width_us=(sg90_min_pulse_width_us+(sg90_max_pulse_width_us-sg90_min_pulse_width_us)*(angle /180));//将0度和180度映射到 500~2500的脉冲上,返回的结果为高电平持续的时间：单位us
    //占空比= 高电平持续时间 (ps:单位转换为s  1s=10^6us) / 一个周期的时间(1/sg90_freq_hz) (ps单位：秒) 

    //高电平持续时间 =pulse_width_us ，一个周期的时间 (1/sg90_freq_hz)     pulse_width_us*10^-6/(1/sg90_freq_hz) *=pulse_width_us *sg90_freq_hz /10^6(1e6)
   //为什么要乘以 1<<13 -1 即 8192呢，因为13位的分辨率 需要将计算得到的占空比值缩放至 0 到 (1<<13) - 1 的范围内。
    duty=pulse_width_us *sg90_freq_hz *((1<<13)-1)/ 1e6;
    sg90_SetDuty(duty);//设置占空比

}

