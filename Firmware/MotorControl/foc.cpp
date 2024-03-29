
#include "foc.hpp"
#include <board.h>

Motor::Error AlphaBetaFrameController::on_measurement(
            std::optional<float> vbus_voltage,
            std::optional<std::array<float, 3>> currents,
            uint32_t input_timestamp) 
{

    std::optional<float2D> Ialpha_beta;
    
    if (currents.has_value()) 
    {
        // Clarke transform
        Ialpha_beta = 
        {            
            (*currents)[0],
            one_by_sqrt3 * ((*currents)[1] - (*currents)[2])
        };
    }
    
    return on_measurement(vbus_voltage, Ialpha_beta, input_timestamp);
}

// 计算FOC,获得PWM. 用来控制电机转动
// @sa Motor::pwm_update_cb
Motor::Error AlphaBetaFrameController::get_output(
            uint32_t output_timestamp, float (&pwm_timings)[3],
            std::optional<float>* ibus) 
{
    std::optional<float2D> mod_alpha_beta;
    Motor::Error status = get_alpha_beta_output(output_timestamp, &mod_alpha_beta, ibus);
    
    if (status != Motor::ERROR_NONE) 
    {
        return status;
    }
    else if (!mod_alpha_beta.has_value() || is_nan(mod_alpha_beta->first) || is_nan(mod_alpha_beta->second)) 
    {
        return Motor::ERROR_MODULATION_IS_NAN;
    }

    // 转SVPWM alpha and beta to ta tb tc
    auto [tA, tB, tC, success] = SVM(mod_alpha_beta->first, mod_alpha_beta->second);
    if (!success) 
    {
        return Motor::ERROR_MODULATION_MAGNITUDE;
    }

    pwm_timings[0] = tA;
    pwm_timings[1] = tB;
    pwm_timings[2] = tC;

    return Motor::ERROR_NONE;
}

void FieldOrientedController::reset() 
{
    v_current_control_integral_d_ = 0.0f;
    v_current_control_integral_q_ = 0.0f;
    vbus_voltage_measured_ = std::nullopt;
    Ialpha_beta_measured_ = std::nullopt;
    power_ = 0.0f;
}

Motor::Error FieldOrientedController::on_measurement(
        std::optional<float> vbus_voltage, std::optional<float2D> Ialpha_beta,
        uint32_t input_timestamp) 
{
    // Store the measurements for later processing.
    i_timestamp_ = input_timestamp;// 对应着相电流被采集时的时间戳
    vbus_voltage_measured_ = vbus_voltage;
    Ialpha_beta_measured_ = Ialpha_beta;

    return Motor::ERROR_NONE;
}

// 这里计算出最后的 mod_alpha_beta   给PWM使用
// @sa AlphaBetaFrameController::get_output
// @sa Motor::pwm_update_cb
ODriveIntf::MotorIntf::Error FieldOrientedController::get_alpha_beta_output(
        uint32_t output_timestamp, std::optional<float2D>* mod_alpha_beta,
        std::optional<float>* ibus) 
{
    // 不能在电机未初始化情况下运行这个函数
    if (!vbus_voltage_measured_.has_value() || !Ialpha_beta_measured_.has_value()) 
    {
        // FOC didn't receive a current measurement yet.
        return Motor::ERROR_CONTROLLER_INITIALIZING;
    } 
    else if (abs((int32_t)(i_timestamp_ - ctrl_timestamp_)) > MAX_CONTROL_LOOP_UPDATE_TO_CURRENT_UPDATE_DELTA) 
    {
        // Data from control loop and current measurement are too far apart.
        // 检测时间超时  
        // 正常情况下 i_timestamp_ 和 ctrl_timestamp_是一样大小。
        // 实际上Odrive的设计，它认为“ctrl_timestamp_”是一个即时更新的量，
        // 它对应着“phase”计算出来的时间点，只不过目前代码中简化了，
        // 让它等于了“i_timestamp_”，它应该要比“i_timestamp_”时间戳晚一点。
        // 因为：先是采集到了电流（对应“i_timestamp_”），然后各种update获取phase等值
        // （对应“ctrl_timestamp_”），而调用到这里的时候，是因为“pwm_update_cb”的调用引起的，
        // 时间又过去了一小会了。这个分析很重要，否则后面的Park转换时的phase的计算就会迷惑。
        return Motor::ERROR_BAD_TIMING;
    }

    // ctrl_timestamp_只有一个地方进行了更新 control_loop_cb
    // 
    // TODO: improve efficiency in case PWM updates are requested at a higher
    // rate than current sensor updates. In this case we can reuse mod_d and
    // mod_q from a previous iteration.

    if (!Vdq_setpoint_.has_value()) 
    {
        return Motor::ERROR_UNKNOWN_VOLTAGE_COMMAND;
    } else if (!phase_.has_value() || !phase_vel_.has_value()) 
    {
        return Motor::ERROR_UNKNOWN_PHASE_ESTIMATE;
    } else if (!vbus_voltage_measured_.has_value()) 
    {
        return Motor::ERROR_UNKNOWN_VBUS_VOLTAGE;
    }

    auto [Vd, Vq] = *Vdq_setpoint_;
    float phase = *phase_;
    float phase_vel = *phase_vel_;
    float vbus_voltage = *vbus_voltage_measured_;

    std::optional<float2D> Idq;

    // Park transform park变换
    if (Ialpha_beta_measured_.has_value()) 
    {
        auto [Ialpha, Ibeta] = *Ialpha_beta_measured_;
        // “phase_”的update获取，在电流的获取时间后面，它对应着时间戳“ctrl_timestamp_”， 
        // 而电流的获取对应着时间戳“i_timestamp_”，
        // 那么我们计算的时候显然要获的在电流采集时这个“瞬间”的“电角度”，
        // 这个“电角度”显然比起“phase_”要前一些
        float I_phase = phase + phase_vel * ((float)(int32_t)(i_timestamp_ - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
        float c_I = our_arm_cos_f32(I_phase);
        float s_I = our_arm_sin_f32(I_phase);
        Idq = 
        {
            c_I * Ialpha + s_I * Ibeta,
            c_I * Ibeta - s_I * Ialpha
        };
        Id_measured_ += I_measured_report_filter_k_ * (Idq->first - Id_measured_);
        Iq_measured_ += I_measured_report_filter_k_ * (Idq->second - Iq_measured_);
    } else 
    {
        Id_measured_ = 0.0f;
        Iq_measured_ = 0.0f;
    }


    float mod_to_V = (2.0f / 3.0f) * vbus_voltage;// 2/3的电压
    float V_to_mod = 1.0f / mod_to_V;
    float mod_d;
    float mod_q;

    if (enable_current_control_) 
    {
        // Current control mode
        if (!pi_gains_.has_value()) 
        {
            return Motor::ERROR_UNKNOWN_GAINS;
        } else if (!Idq.has_value()) 
        {
            return Motor::ERROR_UNKNOWN_CURRENT_MEASUREMENT;
        } else if (!Idq_setpoint_.has_value()) 
        {
            return Motor::ERROR_UNKNOWN_CURRENT_COMMAND;
        }

        // motor::update_current_controller_gains  计算pi_gains_
        auto [p_gain, i_gain] = *pi_gains_;// p_gain=0.014 i_gain = 82.492 // 电流PI环
        auto [Id, Iq] = *Idq;// 上面park变换得到的
        auto [Id_setpoint, Iq_setpoint] = *Idq_setpoint_;// 这个由Motor::update计算得到

        float Ierr_d = Id_setpoint - Id;// setpoint - park(measure)
        float Ierr_q = Iq_setpoint - Iq;

        // Apply PI control (V{d,q}_setpoint act as feed-forward terms in this mode)
        // v_current_control_integral_d_ 可以被监控到 
        // Vd = 0  “mod_d”和“mod_q”是一个针对2/3Ud 的“比值”
        mod_d = V_to_mod * (Vd + v_current_control_integral_d_ + Ierr_d * p_gain);// p_gain=0.014
        mod_q = V_to_mod * (Vq + v_current_control_integral_q_ + Ierr_q * p_gain);

        // Vector modulation saturation, lock integrator if saturated
        // TODO make maximum modulation configurable
        float mod_scalefactor = 0.80f * sqrt3_by_2 * 1.0f / std::sqrt(mod_d * mod_d + mod_q * mod_q);
        if (mod_scalefactor < 1.0f) 
        {
            mod_d *= mod_scalefactor;
            mod_q *= mod_scalefactor;
            // TODO make decayfactor configurable
            v_current_control_integral_d_ *= 0.99f;
            v_current_control_integral_q_ *= 0.99f;
        } 
        else 
        {
            v_current_control_integral_d_ += Ierr_d * (i_gain * current_meas_period);
            v_current_control_integral_q_ += Ierr_q * (i_gain * current_meas_period);
        }

    }
    else 
    {
        // Voltage control mode
        mod_d = V_to_mod * Vd;
        mod_q = V_to_mod * Vq;
    }

    // Inverse park transform 反park变换
    float pwm_phase = phase + phase_vel * ((float)(int32_t)(output_timestamp - ctrl_timestamp_) / (float)TIM_1_8_CLOCK_HZ);
    float c_p = our_arm_cos_f32(pwm_phase);
    float s_p = our_arm_sin_f32(pwm_phase);
    float mod_alpha = c_p * mod_d - s_p * mod_q;// 注意这里出的mod_alpha mod_beta和 final_v_alpha_的关联
    float mod_beta = c_p * mod_q + s_p * mod_d;

    // Report final applied voltage in stationary frame (for sensorless estimator)
    final_v_alpha_ = mod_to_V * mod_alpha;// 2/3电压下的mod_alpha  可以观察到
    final_v_beta_ = mod_to_V * mod_beta;

    *mod_alpha_beta = {mod_alpha, mod_beta};

    if (Idq.has_value()) 
    {
        auto [Id, Iq] = *Idq;
        *ibus = mod_d * Id + mod_q * Iq;
        power_ = vbus_voltage * (*ibus).value();
    }
    
    return Motor::ERROR_NONE;
}

void FieldOrientedController::update(uint32_t timestamp) 
{
    CRITICAL_SECTION() {
        ctrl_timestamp_ = timestamp;
        enable_current_control_ = enable_current_control_src_;
        Idq_setpoint_ = Idq_setpoint_src_.present();// = motor_.Idq_setpoint_  = id,iq
        Vdq_setpoint_ = Vdq_setpoint_src_.present();//= motor_.Vdq_setpoint_ = vd,vq
        phase_ = phase_src_.present();// encoder update下计算
        phase_vel_ = phase_vel_src_.present(); // encoder update下计算
    }
}
