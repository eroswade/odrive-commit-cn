
#include "odrive_main.h"
#include <algorithm>

bool Controller::apply_config() {
    config_.parent = this;
    update_filter_gains();
    return true;
}

void Controller::reset() 
{
    // pos_setpoint is initialized in start_closed_loop_control
    vel_setpoint_ = 0.0f;
    vel_integrator_torque_ = 0.0f;
    torque_setpoint_ = 0.0f;
    mechanical_power_ = 0.0f;
    electrical_power_ = 0.0f;
}

void Controller::set_error(Error error) 
{
    error_ |= error;
    last_error_time_ = odrv.n_evt_control_loop_ * current_meas_period;
}

//--------------------------------
// Command Handling
//--------------------------------

// 控制器, 移动到一个位置
// 调用了加速模型里的算法. 算法修改了Tf_ Xi_ Xf_ Vi_ yAccel_  由trap_traj_自己维护
void Controller::move_to_pos(float goal_point) 
{
    axis_->trap_traj_.planTrapezoidal(goal_point, pos_setpoint_, vel_setpoint_,
                                 axis_->trap_traj_.config_.vel_limit,
                                 axis_->trap_traj_.config_.accel_limit,
                                 axis_->trap_traj_.config_.decel_limit);
    axis_->trap_traj_.t_ = 0.0f;
    trajectory_done_ = false;
}

// 增量运行
// @param: displacement 位移量
// @param: from_input_pos 是否为输入量
void Controller::move_incremental(float displacement, bool from_input_pos = true)
{
    if(from_input_pos)// 是输入
    {
        input_pos_ += displacement; // 增加增加一个位移量
    } 
    else
    {
        input_pos_ = pos_setpoint_ + displacement; // 否则, pos_setpoint_+displacement 绝对位移.
    }

    input_pos_updated();
}

void Controller::start_anticogging_calibration() 
{
    // Ensure the cogging map was correctly allocated earlier and that the motor is capable of calibrating
    if (axis_->error_ == Axis::ERROR_NONE) {
        config_.anticogging.calib_anticogging = true;
    }
}


/*
 * This anti-cogging implementation iterates through each encoder position,
 * waits for zero velocity & position error,
 * then samples the current required to maintain that position.
 * 
 * This holding current is added as a feedforward term in the control loop.
 */
bool Controller::anticogging_calibration(float pos_estimate, float vel_estimate) 
{
    float pos_err = input_pos_ - pos_estimate;
    if (std::abs(pos_err) <= config_.anticogging.calib_pos_threshold / (float)axis_->encoder_.config_.cpr &&
        std::abs(vel_estimate) < config_.anticogging.calib_vel_threshold / (float)axis_->encoder_.config_.cpr) {
        config_.anticogging.cogging_map[std::clamp<uint32_t>(config_.anticogging.index++, 0, 3600)] = vel_integrator_torque_;
    }
    if (config_.anticogging.index < 3600) 
    {
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = config_.anticogging.index * axis_->encoder_.getCoggingRatio();
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        return false;
    } else 
    {
        config_.anticogging.index = 0;
        config_.control_mode = CONTROL_MODE_POSITION_CONTROL;
        input_pos_ = 0.0f;  // Send the motor home
        input_vel_ = 0.0f;
        input_torque_ = 0.0f;
        input_pos_updated();
        anticogging_valid_ = true;
        config_.anticogging.calib_anticogging = false;
        return true;
    }
}

// 设置输入位置
void Controller::set_input_pos_and_steps(float const pos) 
{
    input_pos_ = pos;
    if (config_.circular_setpoints) 
    {
        // 如果循环位置模式, 求模
        float const range = config_.circular_setpoint_range;
        axis_->steps_ = (int64_t)(fmodf_pos(pos, range) / range * config_.steps_per_circular_range);
    } else 
    {
        // 普通位置模式,直接给出 steps_
        axis_->steps_ = (int64_t)(pos * config_.steps_per_circular_range);
    }
}

bool Controller::control_mode_updated() 
{
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) 
    {
        std::optional<float> estimate = (config_.circular_setpoints ?
                                pos_estimate_circular_src_ :
                                pos_estimate_linear_src_).any();
        if (!estimate.has_value()) {
            return false;
        }

        pos_setpoint_ = *estimate;
        set_input_pos_and_steps(*estimate);
    }
    return true;
}


void Controller::update_filter_gains() {
    // input_filter_bandwidth = 2
    float bandwidth = std::min(config_.input_filter_bandwidth, 0.25f * current_meas_hz);// 最后选择的是2
    // 4
    input_filter_ki_ = 2.0f * bandwidth;  // basic conversion to discrete time 2*2=4
    // 16*0.25=4
    input_filter_kp_ = 0.25f * (input_filter_ki_ * input_filter_ki_); // Critically damped
}

static float limitVel(const float vel_limit, const float vel_estimate, const float vel_gain, const float torque) {
    float Tmax = (vel_limit - vel_estimate) * vel_gain;
    float Tmin = (-vel_limit - vel_estimate) * vel_gain;
    return std::clamp(torque, Tmin, Tmax);
}

bool Controller::update() 
{
    std::optional<float> pos_estimate_linear = pos_estimate_linear_src_.present();
    std::optional<float> pos_estimate_circular = pos_estimate_circular_src_.present();
    std::optional<float> pos_wrap = pos_wrap_src_.present();
    std::optional<float> vel_estimate = vel_estimate_src_.present();

    std::optional<float> anticogging_pos_estimate = axis_->encoder_.pos_estimate_.present();
    std::optional<float> anticogging_vel_estimate = axis_->encoder_.vel_estimate_.present();

    if (axis_->step_dir_active_) // 如果是步进模式. 就是数脉冲模式
    {
        if (config_.circular_setpoints) //循环位置模式
        {
            if (!pos_wrap.has_value()) // 位置范围已经设定 
            {
                set_error(ERROR_INVALID_CIRCULAR_RANGE);
                return false;
            }
            input_pos_ = (float)(axis_->steps_ % config_.steps_per_circular_range) * (*pos_wrap / (float)(config_.steps_per_circular_range));
        } 
        else 
        {
            // 得到位置
            input_pos_ = (float)(axis_->steps_) / (float)(config_.steps_per_circular_range); 
        }
    }

    if (config_.anticogging.calib_anticogging) 
    {
        // calibration
        if (!anticogging_pos_estimate.has_value() || !anticogging_vel_estimate.has_value()) 
        {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        // non-blocking
        anticogging_calibration(*anticogging_pos_estimate, *anticogging_vel_estimate);
    }

    // TODO also enable circular deltas for 2nd order filter, etc.
    // 如果有位置限制,设置位置限制. 这个在环形控制会有
    if (config_.circular_setpoints) 
    {
        if (!pos_wrap.has_value()) // 和脉冲模式做了一样的处理
        {
            set_error(ERROR_INVALID_CIRCULAR_RANGE);
            return false;
        }
        input_pos_ = fmodf_pos(input_pos_, *pos_wrap);
    }

    // Update inputs
    switch (config_.input_mode) 
    {
        case INPUT_MODE_INACTIVE: {
            // do nothing 不动
        } break;
        case INPUT_MODE_PASSTHROUGH: 
        {
            // 只有在passthourgh 的时候. input_pos = pos_setpoint
            // 也就是正常情况下, 应该调用pos_setpoint??
            pos_setpoint_ = input_pos_;
            vel_setpoint_ = input_vel_;
            torque_setpoint_ = input_torque_; 
        } break;
        case INPUT_MODE_VEL_RAMP: // 转速爬升控制模式
        {
            // 转速爬升速度 axis.controller.config.vel_ramp_rate = 0.5 [turn/s^2]
            // 
            float max_step_size = std::abs(current_meas_period * config_.vel_ramp_rate);//vel_ramp_rate默认为1
            float full_step = input_vel_ - vel_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            vel_setpoint_ += step;
            // Keep in mind this will need to change with the load / mass
            torque_setpoint_ = (step / current_meas_period) * config_.inertia;// config_.inertia 默认为0 
        } break;
        case INPUT_MODE_TORQUE_RAMP: // 转距爬升控制
        {
            float max_step_size = std::abs(current_meas_period * config_.torque_ramp_rate);//torque_ramp_rate默认为0.1
            float full_step = input_torque_ - torque_setpoint_;
            float step = std::clamp(full_step, -max_step_size, max_step_size);

            torque_setpoint_ += step;
        } break;
        case INPUT_MODE_POS_FILTER: // 平滑位置模式
        {
            // 当使用直接位置控制时由于输入的位置值直接赋予到 ODrive 中，
            // 所以运转会不够平顺。即使使用外部控制器生成好的轨迹位置，
            // 当外部控制器以一定频率向 ODrive 发送位置指令时某些状态下还是会不够平滑。
            // 这时候使用平滑位置控制模式会是不错的选择，推荐将输入滤波带宽设置为位置指令发送频率的一半。
            // 2nd order pos tracking filter
            float delta_pos = input_pos_ - pos_setpoint_; // Pos error
            if (config_.circular_setpoints) 
            {
                if (!pos_wrap.has_value()) 
                {
                    set_error(ERROR_INVALID_CIRCULAR_RANGE);
                    return false;
                }
                delta_pos = wrap_pm(delta_pos, *pos_wrap);
            }
            float delta_vel = input_vel_ - vel_setpoint_; // Vel error
            float accel = input_filter_kp_*delta_pos + input_filter_ki_*delta_vel; // Feedback
            torque_setpoint_ = accel * config_.inertia; // Accel inertia=0
            vel_setpoint_ += current_meas_period * accel; // delta vel
            pos_setpoint_ += current_meas_period * vel_setpoint_; // Delta pos
        } break;
        case INPUT_MODE_MIRROR: //镜像
        {
            if (config_.axis_to_mirror < AXIS_COUNT) 
            {
                std::optional<float> other_pos = axes[config_.axis_to_mirror].encoder_.pos_estimate_.present();
                std::optional<float> other_vel = axes[config_.axis_to_mirror].encoder_.vel_estimate_.present();
                std::optional<float> other_torque = axes[config_.axis_to_mirror].controller_.torque_output_.present();

                if (!other_pos.has_value() || !other_vel.has_value() || !other_torque.has_value()) 
                {
                    set_error(ERROR_INVALID_ESTIMATE);
                    return false;
                }

                pos_setpoint_ = *other_pos * config_.mirror_ratio;
                vel_setpoint_ = *other_vel * config_.mirror_ratio;
                torque_setpoint_ = *other_torque * config_.torque_mirror_ratio;
            } 
            else 
            {
                set_error(ERROR_INVALID_MIRROR_AXIS);
                return false;
            }
        } break;
        // case INPUT_MODE_MIX_CHANNELS: {
        //     // NOT YET IMPLEMENTED
        // } break;
        case INPUT_MODE_TRAP_TRAJ: 
        {
            if(input_pos_updated_)// 如果目标位置有变化
            {
                move_to_pos(input_pos_);// 设置目标位置
                input_pos_updated_ = false;
            }
            // Avoid updating uninitialized trajectory
            // 如果位置运行已经结束,不再继续运行
            if (trajectory_done_)
                break;
            
            if (axis_->trap_traj_.t_ > axis_->trap_traj_.Tf_) // 如果运行时长大于全时长. 停止
            {
                // Drop into position control mode when done to avoid problems on loop counter delta overflow
                config_.control_mode = CONTROL_MODE_POSITION_CONTROL; // 进入位置保持模式?
                pos_setpoint_ = axis_->trap_traj_.Xf_; // 设置当前位置为 最终运行目标
                vel_setpoint_ = 0.0f;
                torque_setpoint_ = 0.0f;
                trajectory_done_ = true;
            } 
            else 
            {
                TrapezoidalTrajectory::Step_t traj_step = axis_->trap_traj_.eval(axis_->trap_traj_.t_);
                pos_setpoint_ = traj_step.Y;
                vel_setpoint_ = traj_step.Yd;
                torque_setpoint_ = traj_step.Ydd * config_.inertia;
                axis_->trap_traj_.t_ += current_meas_period;
            }
            anticogging_pos_estimate = pos_setpoint_; // FF the position setpoint instead of the pos_estimate
        } break;
        case INPUT_MODE_TUNING: 
        {
            // CONTROL_MODE_POSITION_CONTROL 位置模式, 位置sine
            // CONTROL_MODE_VELOCITY_CONTROL 速度模式  速度sine
            autotuning_phase_ = wrap_pm_pi(autotuning_phase_ + (2.0f * M_PI * autotuning_.frequency * current_meas_period));
            float c = our_arm_cos_f32(autotuning_phase_);
            float s = our_arm_sin_f32(autotuning_phase_);
            pos_setpoint_ = input_pos_ + autotuning_.pos_amplitude * s; // + pos_amp_c * c
            vel_setpoint_ = input_vel_ + autotuning_.vel_amplitude * c;
            torque_setpoint_ = input_torque_ + autotuning_.torque_amplitude * -s;
        } break;
        default: {
            set_error(ERROR_INVALID_INPUT_MODE);
            return false;
        }
        
    }

    // Position control 位置控制
    // TODO Decide if we want to use encoder or pll position here
    float gain_scheduling_multiplier = 1.0f;
    float vel_des = vel_setpoint_;// 初始化目标速度
    if (config_.control_mode >= CONTROL_MODE_POSITION_CONTROL) 
    {
        float pos_err;

        if (config_.circular_setpoints) // 如果是圈模式
        {
            if (!pos_estimate_circular.has_value() || !pos_wrap.has_value()) 
            {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // Keep pos setpoint from drifting
            pos_setpoint_ = fmodf_pos(pos_setpoint_, *pos_wrap);// 计算绝对圈
            // Circular delta
            pos_err = pos_setpoint_ - *pos_estimate_circular; // 计算和目标位置的差值
            pos_err = wrap_pm(pos_err, *pos_wrap);
        } 
        else 
        {
            // 普通的位置模式
            if (!pos_estimate_linear.has_value()) 
            {
                set_error(ERROR_INVALID_ESTIMATE);
                return false;
            }
            // 当前设置位置-传感器位置
            pos_err = pos_setpoint_ - *pos_estimate_linear;//pos_estimate_linear=ax->encoder_.pos_estimate_  由传感器返回的位置
        }

        vel_des += config_.pos_gain * pos_err; // 求得变速 pos_gain=30由自己设置 得到的error为圈值.不是脉冲值
        // V-shaped gain shedule based on position error
        float abs_pos_err = std::abs(pos_err);
        if (config_.enable_gain_scheduling && abs_pos_err <= config_.gain_scheduling_width) //里面不运行
        {
            gain_scheduling_multiplier = abs_pos_err / config_.gain_scheduling_width;
        }
    }

    // Velocity limiting 速度限制 检测vel_des有没有超速度范围
    float vel_lim = config_.vel_limit;// vel_limit=50
    if (config_.enable_vel_limit) //true
    {
        vel_des = std::clamp(vel_des, -vel_lim, vel_lim);//设置速度为+- 50之间
    }

    // Check for overspeed fault (done in this module (controller) for cohesion with vel_lim)
    // 超速出错停止
    if (config_.enable_overspeed_error) // 检测overspeed
    {  
        if (!vel_estimate.has_value()) //encoder检测到的速度
        {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        if (std::abs(*vel_estimate) > config_.vel_limit_tolerance * vel_lim) // 如果这个速度超限最大速度的1.2 也就是60
        {
            set_error(ERROR_OVERSPEED);
            return false;
        }
    }

    // TODO: Change to controller working in torque units
    // Torque per amp gain scheduling (ACIM) 交流感应电机
    float vel_gain = config_.vel_gain;// 0.02
    float vel_integrator_gain = config_.vel_integrator_gain;// 0.2
    if (axis_->motor_.config_.motor_type == Motor::MOTOR_TYPE_ACIM) //不执行
    {
        float effective_flux = axis_->acim_estimator_.rotor_flux_;
        float minflux = axis_->motor_.config_.acim_gain_min_flux;
        if (std::abs(effective_flux) < minflux)
            effective_flux = std::copysignf(minflux, effective_flux);
        vel_gain /= effective_flux;
        vel_integrator_gain /= effective_flux;
        // TODO: also scale the integral value which is also changing units.
        // (or again just do control in torque units)
    }

    // Velocity control 速度控制
    float torque = torque_setpoint_;

    // Anti-cogging is enabled after calibration 
    // We get the current position and apply a current feed-forward
    // ensuring that we handle negative encoder positions properly (-1 == motor->encoder.encoder_cpr - 1)
    // 以下不知道什么时候运行, 但运行位置模式时不运行
    if (anticogging_valid_ && config_.anticogging.anticogging_enabled) // anticogging_enabled=true anticogging_valid_=false
    {
        if (!anticogging_pos_estimate.has_value()) 
        {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        float anticogging_pos = *anticogging_pos_estimate / axis_->encoder_.getCoggingRatio();
        torque += config_.anticogging.cogging_map[std::clamp(mod((int)anticogging_pos, 3600), 0, 3600)];
    }

    float v_err = 0.0f;
    if (config_.control_mode >= CONTROL_MODE_VELOCITY_CONTROL) 
    {
        if (!vel_estimate.has_value()) 
        {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }

        v_err = vel_des - *vel_estimate;// 传感器速度和目标速度得差值
        // gain_scheduling_multiplier默认为1. 看上面的代码.
        // vel_gain = 0.02 速度环增加值为差值的0.02
        torque += (vel_gain * gain_scheduling_multiplier) * v_err;// 修改力值. ==> 变为电  

        // Velocity integral action before limiting 增加速度插值
        torque += vel_integrator_torque_; // 0.2 默认再增加0.2的力距值??
    }

    // Velocity limiting in current mode 如果直接设置就是力距模式, 就需要看一下,是不是超限制了
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL && config_.enable_torque_mode_vel_limit) 
    {
        if (!vel_estimate.has_value()) 
        {
            set_error(ERROR_INVALID_ESTIMATE);
            return false;
        }
        torque = limitVel(config_.vel_limit, *vel_estimate, vel_gain, torque);
    }

    // Torque limiting 扭矩限制
    bool limited = false;
    float Tlim = axis_->motor_.max_available_torque();//找到最大的力距限制 不超15
    if (torque > Tlim) 
    {
        limited = true;
        torque = Tlim;
    }
    if (torque < -Tlim) 
    {
        limited = true;
        torque = -Tlim;
    }

    // Velocity integrator (behaviour dependent on limiting) 速度积分
    if (config_.control_mode < CONTROL_MODE_VELOCITY_CONTROL) // 如果只是力距模式 vel_integrator_torque_=0
    {
        // reset integral if not in use
        vel_integrator_torque_ = 0.0f;
    } 
    else 
    {
        if (limited) // 如果超限 vel_integrator_torque_ 减少1%
        {
            // TODO make decayfactor configurable
            vel_integrator_torque_ *= 0.99f;
        } 
        else  // 正常的限制 速度差*vel_integrator_gain(0.2)
        {
            vel_integrator_torque_ += ((vel_integrator_gain * gain_scheduling_multiplier) * current_meas_period) * v_err;
        }
        // integrator limiting to prevent windup 
        // 得到vel_integrator_torque_  做最大值限制
        vel_integrator_torque_ = std::clamp(vel_integrator_torque_, -config_.vel_integrator_limit, config_.vel_integrator_limit);
    }

    float ideal_electrical_power = 0.0f;
    if (axis_->motor_.config_.motor_type != Motor::MOTOR_TYPE_GIMBAL) // 云台电机
    {
        // 不是云台电机. 计算静止的电流 phase_resistance=0.082 power_ = 12V*(Idq = mod_d * Id + mod_q * Iq)
        ideal_electrical_power = axis_->motor_.current_control_.power_ - \
            SQ(axis_->motor_.current_control_.Iq_measured_) * 1.5f * axis_->motor_.config_.phase_resistance - \
            SQ(axis_->motor_.current_control_.Id_measured_) * 1.5f * axis_->motor_.config_.phase_resistance;
    }
    else 
    {
        // 是云台电机, 直接给current_control_.power_
        ideal_electrical_power = axis_->motor_.current_control_.power_;
    }
    mechanical_power_ += config_.mechanical_power_bandwidth * current_meas_period * (torque * *vel_estimate * M_PI * 2.0f - mechanical_power_);
    electrical_power_ += config_.electrical_power_bandwidth * current_meas_period * (ideal_electrical_power - electrical_power_);

    // Spinout check
    // If mechanical power is negative (braking) and measured power is positive, something is wrong
    // This indicates that the controller is trying to stop, but torque is being produced.
    // Usually caused by an incorrect encoder offset
    if (mechanical_power_ < config_.spinout_mechanical_power_threshold && 
        electrical_power_ > config_.spinout_electrical_power_threshold) 
    {
        set_error(ERROR_SPINOUT_DETECTED);
        return false;
    }

    torque_output_ = torque;//得到最后的力

    // TODO: this is inconsistent with the other errors which are sticky.
    // However if we make ERROR_INVALID_ESTIMATE sticky then it will be
    // confusing that a normal sequence of motor calibration + encoder
    // calibration would leave the controller in an error state.
    error_ &= ~ERROR_INVALID_ESTIMATE;
    return true;
}
