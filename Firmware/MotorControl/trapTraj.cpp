#include <cmath>
#include "odrive_main.h"
#include "utils.hpp"

// A sign function where input 0 has positive sign (not 0)
float sign_hard(float val) {
    return (std::signbit(val)) ? -1.0f : 1.0f;
}

// Symbol                     Description
// Ta, Tv and Td              Duration of the stages of the AL profile Ta 加速时间 Tv最高速运行时间 Td 减速时间
// Xi and Vi                  Adapted initial conditions for the AL profile 当前位置Xi 和当前速度Vi
// Xf                         Position set-point 目标位置
// s                          Direction (sign) of the trajectory 方向
// Vmax, Amax, Dmax and jmax  Kinematic bounds Vmax:最大速度 Amax:最大加速度 Dmax:最大减速度
// Ar, Dr and Vr              Reached values of acceleration and velocity 带符号的Amax, Dmax, Vmax

bool TrapezoidalTrajectory::planTrapezoidal(float Xf, float Xi, float Vi,
                                            float Vmax, float Amax, float Dmax) {
    float dX = Xf - Xi;  // Distance to travel 要运行的距离
    float stop_dist = (Vi * Vi) / (2.0f * Dmax); // Minimum stopping distance 最小停止距离
    float dXstop = std::copysign(stop_dist, Vi); // Minimum stopping displacement  最小停止位移
    float s = sign_hard(dX - dXstop); // Sign of coast velocity (if any) 速度变换
    Ar_ = s * Amax;  // Maximum Acceleration (signed)
    Dr_ = -s * Dmax; // Maximum Deceleration (signed)
    Vr_ = s * Vmax;  // Maximum Velocity (signed)

    // If we start with a speed faster than cruising, then we need to decel instead of accel
    // aka "double deceleration move" in the paper 如果当前速度大于目标速度 要减速
    if ((s * Vi) > (s * Vr_)) {
        Ar_ = -s * Amax;
    }

    // Time to accel/decel to/from Vr (cruise speed) 加速时间,减速时间计算
    Ta_ = (Vr_ - Vi) / Ar_;
    Td_ = -Vr_ / Dr_;

    // Integral of velocity ramps over the full accel and decel times to get
    // minimum displacement required to reach cuising speed 计算到最终速度的最小加速距离.
    float dXmin = 0.5f*Ta_*(Vr_ + Vi) + 0.5f*Td_*Vr_;

    // Are we displacing enough to reach cruising speed? 
    if (s*dX < s*dXmin) {
        // Short move (triangle profile)  // 距离不够,三角运行
        Vr_ = s * std::sqrt(std::max((Dr_*SQ(Vi) + 2*Ar_*Dr_*dX) / (Dr_ - Ar_), 0.0f));
        Ta_ = std::max(0.0f, (Vr_ - Vi) / Ar_);
        Td_ = std::max(0.0f, -Vr_ / Dr_);
        Tv_ = 0.0f;
    } else {
        // Long move (trapezoidal profile) 距离足够, 梯形
        Tv_ = (dX - dXmin) / Vr_;
    }

    // Fill in the rest of the values used at evaluation-time 把最后的时间计算出来.
    Tf_ = Ta_ + Tv_ + Td_;
    Xi_ = Xi;
    Xf_ = Xf;
    Vi_ = Vi;
    yAccel_ = Xi + Vi*Ta_ + 0.5f*Ar_*SQ(Ta_); // pos at end of accel phase

    return true;
}

TrapezoidalTrajectory::Step_t TrapezoidalTrajectory::eval(float t) {
    Step_t trajStep;
    if (t < 0.0f) {  // Initial Condition 初始化状态
        trajStep.Y   = Xi_;
        trajStep.Yd  = Vi_;
        trajStep.Ydd = 0.0f;
    } else if (t < Ta_) {  // Accelerating 加速
        trajStep.Y   = Xi_ + Vi_*t + 0.5f*Ar_*SQ(t);
        trajStep.Yd  = Vi_ + Ar_*t;
        trajStep.Ydd = Ar_;
    } else if (t < Ta_ + Tv_) {  // Coasting 滑行
        trajStep.Y   = yAccel_ + Vr_*(t - Ta_);
        trajStep.Yd  = Vr_;
        trajStep.Ydd = 0.0f;
    } else if (t < Tf_) {  // Deceleration 减速
        float td     = t - Tf_;
        trajStep.Y   = Xf_ + 0.5f*Dr_*SQ(td);
        trajStep.Yd  = Dr_*td;
        trajStep.Ydd = Dr_;
    } else if (t >= Tf_) {  // Final Condition 最后清空
        trajStep.Y   = Xf_;
        trajStep.Yd  = 0.0f;
        trajStep.Ydd = 0.0f;
    } else {
        // TODO: report error here
    }

    return trajStep;
}