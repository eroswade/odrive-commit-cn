
#define __MAIN_CPP__
#include "odrive_main.h"
#include "nvm_config.hpp"

#include "usart.h"
#include "freertos_vars.h"
#include "usb_device.h"
#include <communication/interface_usb.h>
#include <communication/interface_uart.h>
#include <communication/interface_i2c.h>
#include <communication/interface_can.hpp>

osSemaphoreId sem_usb_irq;
osMessageQId uart_event_queue;
osMessageQId usb_event_queue;
osSemaphoreId sem_can;

#if defined(STM32F405xx)
// Place FreeRTOS heap in core coupled memory for better performance
__attribute__((section(".ccmram")))
#endif
uint8_t ucHeap[configTOTAL_HEAP_SIZE];

uint32_t _reboot_cookie __attribute__ ((section (".noinit")));
extern char _estack; // provided by the linker script


ODrive odrv{};


ConfigManager config_manager;

class StatusLedController {
public:
    void update();
};

StatusLedController status_led_controller;

void StatusLedController::update() 
{
#if HW_VERSION_MAJOR == 4
    uint32_t t = HAL_GetTick();

    bool is_booting = std::any_of(axes.begin(), axes.end(), [](Axis& axis){
        return axis.current_state_ == Axis::AXIS_STATE_UNDEFINED;
    });

    if (is_booting) {
        return;
    }

    bool is_armed = std::any_of(axes.begin(), axes.end(), [](Axis& axis){
        return axis.motor_.is_armed_;
    });
    bool any_error = odrv.any_error();

    if (is_armed) {
        // Fast green pulsating
        const uint32_t period_ms = 256;
        const uint8_t min_brightness = 0;
        const uint8_t max_brightness = 255;
        uint32_t brightness = std::abs((int32_t)(t % period_ms) - (int32_t)(period_ms / 2)) * (max_brightness - min_brightness) / (period_ms / 2) + min_brightness;
        brightness = (brightness * brightness) >> 8; // eye response very roughly sqrt
        status_led.set_color(rgb_t{(uint8_t)(any_error ? brightness / 2 : 0), (uint8_t)brightness, 0});
    } else if (any_error) {
        // Red pulsating
        const uint32_t period_ms = 1024;
        const uint8_t min_brightness = 0;
        const uint8_t max_brightness = 255;
        uint32_t brightness = std::abs((int32_t)(t % period_ms) - (int32_t)(period_ms / 2)) * (max_brightness - min_brightness) / (period_ms / 2) + min_brightness;
        brightness = (brightness * brightness) >> 8; // eye response very roughly sqrt
        status_led.set_color(rgb_t{(uint8_t)brightness, 0, 0});
    } else {
        // Slow blue pulsating
        const uint32_t period_ms = 4096;
        const uint8_t min_brightness = 50;
        const uint8_t max_brightness = 160;
        uint32_t brightness = std::abs((int32_t)(t % period_ms) - (int32_t)(period_ms / 2)) * (max_brightness - min_brightness) / (period_ms / 2) + min_brightness;
        brightness = (brightness * brightness) >> 8; // eye response very roughly sqrt
        status_led.set_color(rgb_t{0, 0, (uint8_t)brightness});
    }
#endif
}

// 读配置
static bool config_read_all() 
{
    bool success = board_read_config() &&
           config_manager.read(&odrv.config_) &&
           config_manager.read(&odrv.can_.config_);
    {
        success = config_manager.read(&encoders.config_) &&
                  config_manager.read(&axes.sensorless_estimator_.config_) &&
                  config_manager.read(&axes.controller_.config_) &&
                  config_manager.read(&axes.trap_traj_.config_) &&
                  config_manager.read(&axes.min_endstop_.config_) &&
                  config_manager.read(&axes.max_endstop_.config_) &&
                  config_manager.read(&axes.mechanical_brake_.config_) &&
                  config_manager.read(&motors.config_) &&
                  config_manager.read(&motors.fet_thermistor_.config_) &&
                  config_manager.read(&motors.motor_thermistor_.config_) &&
                  config_manager.read(&axes.config_);
    }
    return success;
}

// 写配置 save_config调用
static bool config_write_all() 
{
    bool success = board_write_config() &&
           config_manager.write(&odrv.config_) &&
           config_manager.write(&odrv.can_.config_);
    {
        success = config_manager.write(&encoders.config_) &&
                  config_manager.write(&axes.sensorless_estimator_.config_) &&
                  config_manager.write(&axes.controller_.config_) &&
                  config_manager.write(&axes.trap_traj_.config_) &&
                  config_manager.write(&axes.min_endstop_.config_) &&
                  config_manager.write(&axes.max_endstop_.config_) &&
                  config_manager.write(&axes.mechanical_brake_.config_) &&
                  config_manager.write(&motors.config_) &&
                  config_manager.write(&motors.fet_thermistor_.config_) &&
                  config_manager.write(&motors.motor_thermistor_.config_) &&
                  config_manager.write(&axes.config_);
    }
    return success;
}

// 清空配置
static void config_clear_all() 
{
    odrv.config_ = {};
    odrv.can_.config_ = {};
    {
        encoders.config_ = {};
        axes.sensorless_estimator_.config_ = {};
        axes.controller_.config_ = {};
        axes.controller_.config_.load_encoder_axis = 0;
        axes.trap_traj_.config_ = {};
        axes.min_endstop_.config_ = {};
        axes.max_endstop_.config_ = {};
        axes.mechanical_brake_.config_ = {};
        motors.config_ = {};
        motors.fet_thermistor_.config_ = {};
        motors.motor_thermistor_.config_ = {};
        axes.clear_config();
    }
}

static bool config_apply_all() 
{
    bool success = odrv.can_.apply_config();
    {
        success = encoders.apply_config(motors.config_.motor_type)
               && axes.controller_.apply_config()
               && axes.min_endstop_.apply_config()
               && axes.max_endstop_.apply_config()
               && motors.apply_config()
               && motors.motor_thermistor_.apply_config()
               && axes.apply_config();
    }
    return success;
}

// 存配置. 对应odrv0.save_configuration()
bool ODrive::save_configuration(void) 
{
    bool success;

    CRITICAL_SECTION() 
    {
        bool any_armed = axes.motor_.is_armed_;
        if (any_armed) 
        {
            return false;
        }

        size_t config_size = 0;
        success = config_manager.prepare_store()
               && config_write_all()
               && config_manager.start_store(&config_size)
               && config_write_all()
               && config_manager.finish_store();

        // 注意: 在save_configuration的时候,会丢掉一些中断.所以最好重启一下.
        // FIXME: during save_configuration we might miss some interrupts
        // because the CPU gets halted during a flash erase. Missing events
        // (encoder updates, step/dir steps) is not good so to be sure we just
        // reboot.
        NVIC_SystemReset();
    }

    return success;
}

// 删除配置 对应odrv0.erase_configuration()
void ODrive::erase_configuration(void) 
{
    NVM_erase();

    // 这里重启动,NVIC_SystemReset,是防止save_configuration把RAM写到NVM. NVM是一块FLASH. 
    // 这个操作目的是把RAM设为默认值. 但现在不确定,因为一些Startup操作依赖这些配置. 
    // 所以这个操作可能会导致一些崩溃.
    // FIXME: this reboot is a workaround because we don't want the next save_configuration
    // to write back the old configuration from RAM to NVM. The proper action would
    // be to reset the values in RAM to default. However right now that's not
    // practical because several startup actions depend on the config. The
    // other problem is that the stack overflows if we reset to default here.
    NVIC_SystemReset();
}

void ODrive::enter_dfu_mode() 
{
    if ((hw_version_major_ == 3) && (hw_version_minor_ >= 5)) 
    {
        __asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        _reboot_cookie = 0xDEADBEEF;
        NVIC_SystemReset();
    } else {
        /*
        * DFU mode is only allowed on board version >= 3.5 because it can burn
        * the brake resistor FETs on older boards.
        * If you really want to use it on an older board, add 3.3k pull-down resistors
        * to the AUX_L and AUX_H signals and _only then_ uncomment these lines.
        */
        //__asm volatile ("CPSID I\n\t":::"memory"); // disable interrupts
        //_reboot_cookie = 0xDEADFE75;
        //NVIC_SystemReset();
    }
}

bool ODrive::any_error() 
{
    return error_ != ODrive::ERROR_NONE
        ||   axes.error_ != Axis::ERROR_NONE
                || axes.motor_.error_ != Motor::ERROR_NONE
                || axes.sensorless_estimator_.error_ != SensorlessEstimator::ERROR_NONE
                || axes.encoder_.error_ != Encoder::ERROR_NONE
                || axes.controller_.error_ != Controller::ERROR_NONE;
}

uint64_t ODrive::get_drv_fault() 
{
    return motors.gate_driver_.get_error();
}

void ODrive::clear_errors() 
{
        axes.motor_.error_ = Motor::ERROR_NONE;
        axes.controller_.error_ = Controller::ERROR_NONE;
        axes.sensorless_estimator_.error_ = SensorlessEstimator::ERROR_NONE;
        axes.encoder_.error_ = Encoder::ERROR_NONE;
        axes.encoder_.spi_error_rate_ = 0.0f;
        axes.error_ = Axis::ERROR_NONE;
    error_ = ERROR_NONE;
    if (odrv.config_.enable_brake_resistor) 
    {
        safety_critical_arm_brake_resistor();
    }
}

extern "C" 
{

// 程序崩溃后的处理
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed portCHAR *pcTaskName) 
{
        axes.motor_.disarm();
    safety_critical_disarm_brake_resistor();
    for (;;); // TODO: safe action
}

// 空闲处理
void vApplicationIdleHook(void) 
{
    if (odrv.system_stats_.fully_booted) 
    {
        odrv.system_stats_.uptime = xTaskGetTickCount();
        odrv.system_stats_.min_heap_space = xPortGetMinimumEverFreeHeapSize();

        odrv.system_stats_.max_stack_usage_axis = axes.stack_size_ - uxTaskGetStackHighWaterMark(axes.thread_id_) * sizeof(StackType_t); 
        odrv.system_stats_.max_stack_usage_usb = stack_size_usb_thread - uxTaskGetStackHighWaterMark(usb_thread) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_uart = stack_size_uart_thread - uxTaskGetStackHighWaterMark(uart_thread) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_startup = stack_size_default_task - uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_can = odrv.can_.stack_size_ - uxTaskGetStackHighWaterMark(odrv.can_.thread_id_) * sizeof(StackType_t);
        odrv.system_stats_.max_stack_usage_analog =  stack_size_analog_thread - uxTaskGetStackHighWaterMark(analog_thread) * sizeof(StackType_t);

        odrv.system_stats_.stack_size_axis = axes.stack_size_;
        odrv.system_stats_.stack_size_usb = stack_size_usb_thread;
        odrv.system_stats_.stack_size_uart = stack_size_uart_thread;
        odrv.system_stats_.stack_size_startup = stack_size_default_task;
        odrv.system_stats_.stack_size_can = odrv.can_.stack_size_;
        odrv.system_stats_.stack_size_analog = stack_size_analog_thread;

        odrv.system_stats_.prio_axis = osThreadGetPriority(axes.thread_id_);
        odrv.system_stats_.prio_usb = osThreadGetPriority(usb_thread);
        odrv.system_stats_.prio_uart = osThreadGetPriority(uart_thread);
        odrv.system_stats_.prio_startup = osThreadGetPriority(defaultTaskHandle);
        odrv.system_stats_.prio_can = osThreadGetPriority(odrv.can_.thread_id_);
        odrv.system_stats_.prio_analog = osThreadGetPriority(analog_thread);

        status_led_controller.update();
    }
}
}

/**
 * @brief Runs system-level checks that need to be as real-time as possible.
 * 
 * This function is called after every current measurement of every motor.
 * It should finish as quickly as possible.
 * 
 * 运行需要尽可能实时的系统级检查
 * 每次测量每台电机的电流后调用此函数。它应该尽快完成。
 */
void ODrive::do_fast_checks() {
    if (!(vbus_voltage >= config_.dc_bus_undervoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_UNDER_VOLTAGE);
    if (!(vbus_voltage <= config_.dc_bus_overvoltage_trip_level))
        disarm_with_error(ERROR_DC_BUS_OVER_VOLTAGE);
}

/**
 * @brief Floats all power phases on the system (all motors and brake resistors).
 *
 * This should be called if a system level exception ocurred that makes it
 * unsafe to run power through the system in general.
 * 
 * 系统上的所有电源相位（所有电机和制动电阻器）
 * 如果发生系统级异常导致它应该被调用,一般而言，通过系统供电是不安全的。
 */
void ODrive::disarm_with_error(Error error) 
{
    CRITICAL_SECTION() {
            axes.motor_.disarm_with_error(Motor::ERROR_SYSTEM_LEVEL);
        safety_critical_disarm_brake_resistor();
        error_ |= error;
    }
}

/**
 * @brief Runs the periodic sampling tasks
 * 
 * All components that need to sample real-world data should do it in this
 * function as it runs on a high interrupt priority and provides lowest possible
 * timing jitter.
 * 
 * All function called from this function should adhere to the following rules:
 *  - Try to use the same number of CPU cycles in every iteration.
 *    (reason: Tasks that run later in the function still want lowest possible timing jitter)
 *  - Use as few cycles as possible.
 *    (reason: The interrupt blocks other important interrupts (TODO: which ones?))
 *  - Not call any FreeRTOS functions.
 *    (reason: The interrupt priority is higher than the max allowed priority for syscalls)
 * 
 * Time consuming and undeterministic logic/arithmetic should live on
 * control_loop_cb() instead.
 * 
 * 运行定期采样任务
 * 所有需要采样真实世界数据的组件都应该在这做
 * 功能，因为它运行在高中断优先级上并提供尽可能低的中断
 * 定时抖动。
 * 从此函数调用的所有函数都应遵守以下规则：
 * - 尝试在每次迭代中使用相同数量的 CPU 周期。原因：函数中稍后运行的任务仍然需要尽可能低的时序抖动）
 * - 使用尽可能少的周期。（原因：中断阻塞了其他重要的中断（TODO：哪些？））
 * - 不调用任何 FreeRTOS 函数。 （原因：中断优先级高于系统调用的最大允许优先级）
 *  耗时和不确定的逻辑/算术应该继续存在control_loop_cb() 代替
 */
void ODrive::sampling_cb() {
    n_evt_sampling_++;

    MEASURE_TIME(task_times_.sampling) {
            axes.encoder_.sample_now();
    }
}

/**
 * @brief Runs the periodic control loop.
 * 
 * This function is executed in a low priority interrupt context and is allowed
 * to call CMSIS functions.
 * 
 * Yet it runs at a higher priority than communication workloads.
 * 
 * @param update_cnt: The true count of update events (wrapping around at 16
 *        bits). This is used for timestamp calculation in the face of
 *        potentially missed timer update interrupts. Therefore this counter
 *        must not rely on any interrupts.
 * 
 * 周期性运行控制循环
 * 该函数在低优先级中断中执行，允许调用 CMSIS 函数。
 * 然而，它的运行优先级高于通信工作
 * @param update_cnt：更新事件的真实计数（16 位）。这个是用来做时间戳计算的，面对
 * 可能错过定时器更新中断。因此这个柜台
 * 不得依赖任何中断。 
 * 代替以前run_control_loop函数
 */
void ODrive::control_loop_cb(uint32_t timestamp) 
{
    last_update_timestamp_ = timestamp;
    n_evt_control_loop_++;

    // TODO: use a configurable component list for most of the following things
    // 对以下大部分事情使用可配置的组件列表
    MEASURE_TIME(task_times_.control_loop_misc) 
    {
        // Reset all output ports so that we are certain about the freshness of
        // all values that we use.
        // If we forget to reset a value here the worst that can happen is that
        // this safety check doesn't work.
        // TODO: maybe we should add a check to output ports that prevents
        // double-setting the value.
        //重置所有输出端口，以便我们确定
        //我们使用的所有值。
        //如果我们忘记在这里重置一个值，最坏的情况就是
        //此安全检查不起作用。
        // TODO: 也许我们应该对输出端口添加一个检查以防止
        //双重设置值。
        {
            axes.acim_estimator_.slip_vel_.reset();
            axes.acim_estimator_.stator_phase_vel_.reset();
            axes.acim_estimator_.stator_phase_.reset();
            axes.controller_.torque_output_.reset();
            axes.encoder_.phase_.reset();
            axes.encoder_.phase_vel_.reset();
            axes.encoder_.pos_estimate_.reset();
            axes.encoder_.vel_estimate_.reset();
            axes.encoder_.pos_circular_.reset();
            axes.motor_.Vdq_setpoint_.reset();
            axes.motor_.Idq_setpoint_.reset();
            axes.open_loop_controller_.Idq_setpoint_.reset();
            axes.open_loop_controller_.Vdq_setpoint_.reset();
            axes.open_loop_controller_.phase_.reset();
            axes.open_loop_controller_.phase_vel_.reset();
            axes.open_loop_controller_.total_distance_.reset();
            axes.sensorless_estimator_.phase_.reset();
            axes.sensorless_estimator_.phase_vel_.reset();
            axes.sensorless_estimator_.vel_estimate_.reset();
        }

        uart_poll();
        odrv.oscilloscope_.update();
    }

    {// 极限位置检测
        MEASURE_TIME(axes.task_times_.endstop_update) 
        {
            axes.min_endstop_.update();
            axes.max_endstop_.update();
        }
    }

    MEASURE_TIME(task_times_.control_loop_checks) 
    {
            // look for errors at axis level and also all subcomponents 检查有没有出错
            bool checks_ok = axes.do_checks(timestamp);

            // make sure the watchdog is being fed.  喂狗
            bool watchdog_ok = axes.watchdog_check();

            if (!checks_ok || !watchdog_ok) {
                axes.motor_.disarm();
            }
    }

    {
        // Sub-components should use set_error which will propegate to this error_
        // 检测温度
        MEASURE_TIME(axes.task_times_.thermistor_update) 
        {
            axes.motor_.fet_thermistor_.update();// 在板子上的温度传感器
            axes.motor_.motor_thermistor_.update();// 电机上的温度传感器
        }

        MEASURE_TIME(axes.task_times_.encoder_update)
            axes.encoder_.update();
    }

    // Controller of either axis might use the encoder estimate of the other
    // axis so we process both encoders before we continue.

    {
        MEASURE_TIME(axes.task_times_.sensorless_estimator_update)// 如果是无传感器. 更新SensorlessEstimator
            axes.sensorless_estimator_.update();

        MEASURE_TIME(axes.task_times_.controller_update) // 控制器更新
        {
            // 根据控制方式更新torque_setpoint_ vel_setpoint_ pos_setpoint_
            if (!axes.controller_.update()) // uses position and velocity from encoder
            { 
                axes.error_ |= Axis::ERROR_CONTROLLER_FAILED;
            }
        }

        MEASURE_TIME(axes.task_times_.open_loop_controller_update)//开环控制
            axes.open_loop_controller_.update(timestamp);

        MEASURE_TIME(axes.task_times_.motor_update) // 电机扭矩 
            axes.motor_.update(timestamp); // uses torque from controller and phase_vel from encoder

        MEASURE_TIME(axes.task_times_.current_controller_update)// 电流控制
            axes.motor_.current_control_.update(timestamp); // uses the output of controller_ or open_loop_contoller_ and encoder_ or sensorless_estimator_ or acim_estimator_
    }

    // Tell the axis threads that the control loop has finished
    {
        if (axes.thread_id_) 
        {
            osSignalSet(axes.thread_id_, 0x0001);
        }
    }

    get_gpio(odrv.config_.error_gpio_pin).write(odrv.any_error());
}


/** @brief For diagnostics only  仅用于诊断*/
uint32_t ODrive::get_interrupt_status(int32_t irqn) {
    if ((irqn < -14) || (irqn >= 240)) {
        return 0xffffffff;
    }

    uint8_t priority = (irqn < -12)
        ? 0 // hard fault and NMI always have maximum priority
        : NVIC_GetPriority((IRQn_Type)irqn);
    uint32_t counter = GET_IRQ_COUNTER((IRQn_Type)irqn);
    bool is_enabled = (irqn < 0)
        ? true // processor interrupt vectors are always enabled
        : NVIC->ISER[(((uint32_t)(int32_t)irqn) >> 5UL)] & (uint32_t)(1UL << (((uint32_t)(int32_t)irqn) & 0x1FUL));
    
    return priority | ((counter & 0x7ffffff) << 8) | (is_enabled ? 0x80000000 : 0);
}

/** @brief For diagnostics only */
uint32_t ODrive::get_dma_status(uint8_t stream_num) {
    DMA_Stream_TypeDef* streams[] = {
        DMA1_Stream0, DMA1_Stream1, DMA1_Stream2, DMA1_Stream3, DMA1_Stream4, DMA1_Stream5, DMA1_Stream6, DMA1_Stream7,
        DMA2_Stream0, DMA2_Stream1, DMA2_Stream2, DMA2_Stream3, DMA2_Stream4, DMA2_Stream5, DMA2_Stream6, DMA2_Stream7
    };
    if (stream_num >= 16) {
        return 0xffffffff;
    }
    DMA_Stream_TypeDef* stream = streams[stream_num];
    bool is_reset = (stream->CR == 0x00000000)
                 && (stream->NDTR == 0x00000000)
                 && (stream->PAR == 0x00000000)
                 && (stream->M0AR == 0x00000000)
                 && (stream->M1AR == 0x00000000)
                 && (stream->FCR == 0x00000021);
    uint8_t channel = ((stream->CR & DMA_SxCR_CHSEL_Msk) >> DMA_SxCR_CHSEL_Pos);
    uint8_t priority = ((stream->CR & DMA_SxCR_PL_Msk) >> DMA_SxCR_PL_Pos);
    return (is_reset ? 0 : 0x80000000) | ((channel & 0x7) << 2) | (priority & 0x3);
}

uint32_t ODrive::get_gpio_states() {
    // TODO: get values that were sampled synchronously with the control loop 获取与控制循环同步采样的值
    uint32_t val = 0;
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        val |= ((gpios[i].read() ? 1UL : 0UL) << i);
    }
    return val;
}

/**
 * @brief Main thread started from main().  主线程从 main() 开始。
 */
static void rtos_main(void*) {
    // Init USB device 初始化USB设备
    MX_USB_DEVICE_Init();


    // Start ADC for temperature measurements and user measurements
    // 启动 ADC 进行温度测量和用户测量
    start_general_purpose_adc();

    //osDelay(100);
    // Init communications (this requires the axis objects to be constructed)
    // 初始化通信（这需要构建轴对象）
    init_communication();

    // Start pwm-in compare modules 启动 pwm-in 比较模块
    // must happen after communication is initialized  必须在通信初始化之后
    pwm0_input.init();

    // Set up the CS pins for absolute encoders (TODO: move to GPIO init switch statement)
    // 为绝对编码器设置 CS 引脚（TODO：移至 GPIO 初始化 切换）
        if(axes.encoder_.config_.mode & Encoder::MODE_FLAG_ABS){
            axes.encoder_.abs_spi_cs_pin_init();
        }

    // Try to initialized gate drivers for fault-free startup.  尝试初始化栅极驱动器以实现无故障启动
    // If this does not succeed, a fault will be raised and the idle loop will  如果这不成功，将引发错误并且空闲循环
    // periodically attempt to reinit the gate driver. 定期尝试重新初始化栅极驱动器。
        axes.motor_.setup();

        axes.encoder_.setup();

        axes.acim_estimator_.idq_src_.connect_to(&axes.motor_.Idq_setpoint_);

    // Start PWM and enable adc interrupts/callbacks  启动 PWM 并启用 adc 中断/回调
    start_adc_pwm();
    start_analog_thread();

    // Wait for up to 2s for motor to become ready to allow for error-free 最多等待 2 秒，让电机准备就绪，以确保无错误
    // startup. This delay gives the current sensor calibration time to 启动。此延迟使电流传感器校准时间
    // converge. If the DRV chip is unpowered, the motor will not become ready 收敛。如果 DRV 芯片未通电，电机将不会准备就绪
    // but we still enter idle state.
    for (size_t i = 0; i < 2000; ++i) 
    {
        if (axes.motor_.current_meas_.has_value()) {
            break;
        }
        osDelay(1);
    }

        axes.sensorless_estimator_.error_ &= ~SensorlessEstimator::ERROR_UNKNOWN_CURRENT_MEASUREMENT;

    // Start state machine threads. Each thread will go through various calibration 启动状态机线程。每个线程都会经过各种校准
    // procedures and then run the actual controller loops. 程序，然后运行实际的控制器循环。
    // TODO: generalize for AXIS_COUNT != 2 TODO：归纳为 AXIS_COUNT != 2
        axes.start_thread();

    odrv.system_stats_.fully_booted = true;

    // Main thread finished starting everything and can delete itself now (yes this is legal). 主线程完成所有的启动，现在可以删除它自己（是的，这是合法的）。
    vTaskDelete(defaultTaskHandle);
}

/**
 * @brief Carries out early startup tasks that need to run before any static
 * initializers.
 * This function gets called from the startup assembly code.
 * 执行需要在任何静态之前运行的早期启动任务
 */
extern "C" void early_start_checks(void) {
    if(_reboot_cookie == 0xDEADFE75) {
        /* The STM DFU bootloader enables internal pull-up resistors on PB10 (AUX_H)
        * and PB11 (AUX_L), thereby causing shoot-through on the brake resistor
        * FETs and wipe out them unless external 3.3k pull-down resistors are
        * present. Pull-downs are only present on ODrive 3.5 or newer.
        * On older boards we disable DFU by default but if the user insists
        * there's only one thing left that might save it: time.
        * The brake resistor gate driver needs a certain 10V supply (GVDD) to
        * make it work. This voltage is supplied by the motor gate drivers which get
        * disabled at system reset. So over time GVDD voltage _should_ below
        * dangerous levels. This is completely handwavy and should not be relied on
        * so you are on your own on if you ignore this warning.
        *
        * This loop takes 5 cycles per iteration and at this point the system runs
        * on the internal 16MHz RC oscillator so the delay is about 2 seconds.
        */
        for (size_t i = 0; i < (16000000UL / 5UL * 2UL); ++i) {
            __NOP();
        }
        _reboot_cookie = 0xDEADBEEF;
    }

    /* We could jump to the bootloader directly on demand without rebooting
    but that requires us to reset several peripherals and interrupts for it
    to function correctly. Therefore it's easier to just reset the entire chip. */
    if(_reboot_cookie == 0xDEADBEEF) {
        _reboot_cookie = 0xCAFEFEED;  //Reset bootloader trigger
        __set_MSP((uintptr_t)&_estack);
        // http://www.st.com/content/ccc/resource/technical/document/application_note/6a/17/92/02/58/98/45/0c/CD00264379.pdf/files/CD00264379.pdf
        void (*builtin_bootloader)(void) = (void (*)(void))(*((uint32_t *)0x1FFF0004));
        builtin_bootloader();
    }

    /* The bootloader might fail to properly clean up after itself,
    so if we're not sure that the system is in a clean state we
    just reset it again */
    if(_reboot_cookie != 42) {
        _reboot_cookie = 42;
        NVIC_SystemReset();
    }
}

/**
 * @brief Main entry point called from assembly startup code.
 */
extern "C" int main(void) {
    // This procedure of building a USB serial number should be identical
    // to the way the STM's built-in USB bootloader does it. This means
    // that the device will have the same serial number in normal and DFU mode.
    uint32_t uuid0 = *(uint32_t *)(UID_BASE + 0);
    uint32_t uuid1 = *(uint32_t *)(UID_BASE + 4);
    uint32_t uuid2 = *(uint32_t *)(UID_BASE + 8);
    uint32_t uuid_mixed_part = uuid0 + uuid2;
    serial_number = ((uint64_t)uuid_mixed_part << 16) | (uint64_t)(uuid1 >> 16);

    uint64_t val = serial_number;
    for (size_t i = 0; i < 12; ++i) {
        serial_number_str[i] = "0123456789ABCDEF"[(val >> (48-4)) & 0xf];
        val <<= 4;
    }
    serial_number_str[12] = 0;

    // Init low level system functions (clocks, flash interface) 最低系统启动
    system_init();

    // Load configuration from NVM. This needs to happen after system_init()
    // since the flash interface must be initialized and before board_init()
    // since board initialization can depend on the config.
    // 获取配置
    size_t config_size = 0;
    bool success = config_manager.start_load()
            && config_read_all()
            && config_manager.finish_load(&config_size)
            && config_apply_all();
    if (success) {
        odrv.user_config_loaded_ = config_size;
    } else {
        config_clear_all();
        config_apply_all();
    }

    odrv.misconfigured_ = odrv.misconfigured_
            || (odrv.config_.enable_uart_a && !uart_a)
            || (odrv.config_.enable_uart_b && !uart_b)
            || (odrv.config_.enable_uart_c && !uart_c);

    // TIM1 tim8 168Mhz 
    // 其它时钟  84MHZ
    // TIM8 TIM1  168MHz/(3500*2) = 24KHz   因为是中间对齐，所以式子中要*2   period = TIM_1_8_PERIOD_CLOCKS
    // 这里PWM为24KHZ ， 脉冲宽度是3500*2个CYCLE的时长
    // r
    // TIM13 启动时和TIM1 TIM8同步， 任务耗时测量
    // TIM14 1KHZ  HAL库时基
    // TIM2  84M/(4096*2)=10.25KHz  | CH3(PB.10)和CH4(PB.11)作为pwm输出，耗散电阻 
    // tim3 tim4  #1电机之旋编检测，计满到0xfffff  #2电机之旋编检测，计满到0xfffff
    // tim5 定时器的CH3(PA.2)和CH4(PA.3)作为捕获输入口
    // by eros: TODO: can be canceld tim8 tim4 , mast be changed tim13
    // Init board-specific peripherals  初始化板特定的外设
    if (!board_init()) {
        for (;;); // TODO: handle properly
    }

    // Init GPIOs according to their configured mode  根据配置的模式初始化 GPIO
    for (size_t i = 0; i < GPIO_COUNT; ++i) {
        // Skip unavailable GPIOs
        if (!get_gpio(i)) {
            continue;
        }

        ODriveIntf::GpioMode mode = odrv.config_.gpio_modes[i];

        GPIO_InitTypeDef GPIO_InitStruct;
        GPIO_InitStruct.Pin = get_gpio(i).pin_mask_;

        // Set Alternate Function setting for this GPIO mode
        if (mode == ODriveIntf::GPIO_MODE_DIGITAL ||
            mode == ODriveIntf::GPIO_MODE_DIGITAL_PULL_UP ||
            mode == ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN ||
            mode == ODriveIntf::GPIO_MODE_MECH_BRAKE ||
            mode == ODriveIntf::GPIO_MODE_STATUS ||
            mode == ODriveIntf::GPIO_MODE_ANALOG_IN) {
            GPIO_InitStruct.Alternate = 0;
        } else {
            auto it = std::find_if(
                    alternate_functions[i].begin(), alternate_functions[i].end(),
                    [mode](auto a) { return a.mode == mode; });

            if (it == alternate_functions[i].end()) {
                odrv.misconfigured_ = true; // this GPIO doesn't support the selected mode
                continue;
            }
            GPIO_InitStruct.Alternate = it->alternate_function;
        }

        switch (mode) {
            case ODriveIntf::GPIO_MODE_DIGITAL: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_DIGITAL_PULL_UP: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_DIGITAL_PULL_DOWN: {
                GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ANALOG_IN: {
                GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
            } break;
            case ODriveIntf::GPIO_MODE_UART_A: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart_a) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_UART_B: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart_b) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_UART_C: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = (i == 0) ? GPIO_PULLDOWN : GPIO_PULLUP; // this is probably swapped but imitates old behavior
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_uart_c) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_CAN_A: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_can_a) {
                    odrv.misconfigured_ = true;
                }
            } break;
            case ODriveIntf::GPIO_MODE_I2C_A: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_OD;
                GPIO_InitStruct.Pull = GPIO_PULLUP;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
                if (!odrv.config_.enable_i2c_a) {
                    odrv.misconfigured_ = true;
                }
            } break;
            //case ODriveIntf::GPIO_MODE_SPI_A: { // TODO
            //} break;
            case ODriveIntf::GPIO_MODE_PWM: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_PULLDOWN;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC0: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC1: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_ENC2: {
                GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_MECH_BRAKE: {
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            case ODriveIntf::GPIO_MODE_STATUS: {
                GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
                GPIO_InitStruct.Pull = GPIO_NOPULL;
                GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
            } break;
            default: {
                odrv.misconfigured_ = true;
                continue;
            }
        }

        HAL_GPIO_Init(get_gpio(i).port_, &GPIO_InitStruct);
    }

    // Init usb irq binary semaphore, and start with no tokens by removing the starting one.
    osSemaphoreDef(sem_usb_irq);
    sem_usb_irq = osSemaphoreCreate(osSemaphore(sem_usb_irq), 1);
    osSemaphoreWait(sem_usb_irq, 0);

    // Create an event queue for UART 为 UART 创建一个事件队列
    osMessageQDef(uart_event_queue, 4, uint32_t);
    uart_event_queue = osMessageCreate(osMessageQ(uart_event_queue), NULL);

    // Create an event queue for USB 为 USB 创建一个事件队列
    osMessageQDef(usb_event_queue, 7, uint32_t);
    usb_event_queue = osMessageCreate(osMessageQ(usb_event_queue), NULL);

    osSemaphoreDef(sem_can);
    sem_can = osSemaphoreCreate(osSemaphore(sem_can), 1);
    osSemaphoreWait(sem_can, 0);

    // Create main thread  创建主线程
    osThreadDef(defaultTask, rtos_main, osPriorityNormal, 0, stack_size_default_task / sizeof(StackType_t));
    defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

    // Start scheduler 启动调度器
    osKernelStart();
    
    for (;;);
}
