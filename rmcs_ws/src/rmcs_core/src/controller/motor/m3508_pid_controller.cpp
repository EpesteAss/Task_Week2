#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "filter/low_pass_filter.hpp"

namespace rmcs_core::controller::motor {

class M3508PidController
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    M3508PidController()
        : Node(
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true))
        , velocity_filter_(
              get_parameter("filter_cutoff_frequency").as_double(),
              get_parameter("sampling_frequency").as_double()) {

        // DR16
        register_input(
            "/remote/joystick/right",
            joystick_right_);

        // M3508 原始速度反馈
        register_input(
            "/m3508/velocity",
            measurement_);

        // PID 输出
        register_output(
            "/m3508/control_torque",
            control_,
            0.0);

        // PID 参数
        kp_ = get_parameter("kp").as_double();
        ki_ = get_parameter("ki").as_double();
        kd_ = get_parameter("kd").as_double();

        max_velocity_ =
            get_parameter("max_velocity").as_double();

        integral_min_ =
            get_parameter("integral_min").as_double();

        integral_max_ =
            get_parameter("integral_max").as_double();

        output_min_ =
            get_parameter("output_min").as_double();

        output_max_ =
            get_parameter("output_max").as_double();
    }

    void update() override {

        // =========================
        // 1. DR16 → 目标速度
        // =========================

        const double target_velocity =
            joystick_right_->y() * max_velocity_;

        // =========================
        // 2. 电机速度反馈滤波
        // =========================

        const double filtered_velocity =
            velocity_filter_.update(*measurement_);

        if (!std::isfinite(filtered_velocity)) {
            integral_ = 0.0;
            last_error_ = 0.0;
            has_last_error_ = false;

            *control_ = 0.0;
            return;
        }

        // =========================
        // 3. 计算误差
        // =========================

        const double error =
            target_velocity - filtered_velocity;

        // =========================
        // 4. P
        // =========================

        const double p =
            kp_ * error;

        // =========================
        // 5. I
        // =========================

        integral_ += error;

        integral_ =
            std::clamp(
                integral_,
                integral_min_,
                integral_max_);

        const double i =
            ki_ * integral_;

        // =========================
        // 6. D
        // =========================

        double d = 0.0;

        if (has_last_error_) {
            d =
                kd_ *
                (error - last_error_);
        }

        last_error_ = error;
        has_last_error_ = true;

        // =========================
        // 7. PID 输出
        // =========================

        double output =
            p + i + d;

        // =========================
        // 8. 输出限幅
        // =========================

        output =
            std::clamp(
                output,
                output_min_,
                output_max_);

        // =========================
        // 9. 输出给 M3508
        // =========================

        *control_ = output;
        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "joystick_y=%.3f target=%.2f velocity=%.2f output=%.3f", joystick_right_->y(),
            target_velocity, filtered_velocity, *control_);
    }

private:

    // =========================
    // 输入
    // =========================

    InputInterface<Eigen::Vector2d>
        joystick_right_;

    InputInterface<double>
        measurement_;

    // =========================
    // 输出
    // =========================

    OutputInterface<double>
        control_;

    // =========================
    // PID 参数
    // =========================

    double kp_ = 0.0;
    double ki_ = 0.0;
    double kd_ = 0.0;

    double max_velocity_ = 0.0;

    double integral_min_ = -100.0;
    double integral_max_ = 100.0;

    double output_min_ = -10.0;
    double output_max_ = 10.0;

    // =========================
    // PID 状态
    // =========================

    double integral_ = 0.0;
    double last_error_ = 0.0;
    bool has_last_error_ = false;

    // =========================
    // 速度低通滤波器
    // =========================

    rmcs_core::filter::LowPassFilter<1>
        velocity_filter_;
};

}  // namespace rmcs_core::controller::motor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor::M3508PidController,
    rmcs_executor::Component)