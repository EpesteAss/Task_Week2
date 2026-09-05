#include <algorithm>
#include <cmath>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

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

        // =========================
        // RMCS 输入
        // =========================

        register_input(
            "/remote/joystick/right",
            joystick_right_);

        register_input(
            "/m3508/velocity",
            measurement_);

        // =========================
        // RMCS 输出
        // =========================

        register_output(
            "/m3508/control_torque",
            control_,
            0.0);

        // =========================
        // PID 参数
        // =========================

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

        // =========================
        // Foxglove / ROS 2 Topic
        // =========================

        target_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task2/target_velocity",
                10);

        actual_velocity_pub_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task2/actual_velocity",
                10);

        velocity_error_pub_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task2/velocity_error",
                10);

        control_torque_pub_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task2/control_torque",
                10);
    }

    void update() override {

        // RMCS 调度频率：1000 Hz
        constexpr double dt = 0.001;

        // =========================
        // 1. DR16 → 目标速度
        // =========================

        const double target_velocity =
            joystick_right_->y() * max_velocity_;

        // =========================
        // 2. 速度反馈低通滤波
        // =========================

        const double filtered_velocity =
            velocity_filter_.update(*measurement_);

        // =========================
        // 3. 检查反馈
        // =========================

        if (!std::isfinite(filtered_velocity)) {
            integral_ = 0.0;
            last_error_ = 0.0;
            has_last_error_ = false;

            *control_ = 0.0;

            publish_debug_data(
                target_velocity,
                0.0,
                0.0,
                0.0);

            return;
        }

        // =========================
        // 4. 误差
        // =========================

        const double error =
            target_velocity - filtered_velocity;

        // =========================
        // 5. P
        // =========================

        const double p =
            kp_ * error;

        // =========================
        // 6. I
        // =========================

        integral_ += error * dt;

        integral_ =
            std::clamp(
                integral_,
                integral_min_,
                integral_max_);

        const double i =
            ki_ * integral_;

        // =========================
        // 7. D
        // =========================

        double d = 0.0;

        if (has_last_error_) {
            d =
                kd_
                * (error - last_error_)
                / dt;
        }

        last_error_ = error;
        has_last_error_ = true;

        // =========================
        // 8. PID 输出
        // =========================

        double output =
            p + i + d;

        // =========================
        // 9. 输出限幅
        // =========================

        output =
            std::clamp(
                output,
                output_min_,
                output_max_);

        // =========================
        // 10. 输出给 M3508
        // =========================

        *control_ = output;

        // =========================
        // 11. Foxglove 调试
        // 1000 Hz PID → 100 Hz Topic
        // =========================

        publish_debug_data(
            target_velocity,
            filtered_velocity,
            error,
            output);
    }

private:

    void publish_debug_data(
        const double target_velocity,
        const double actual_velocity,
        const double velocity_error,
        const double control_torque) {

        ++debug_publish_counter_;

        if (debug_publish_counter_ < 10)
            return;

        debug_publish_counter_ = 0;

        std_msgs::msg::Float64 target_msg;
        target_msg.data = target_velocity;
        target_velocity_pub_->publish(target_msg);

        std_msgs::msg::Float64 actual_msg;
        actual_msg.data = actual_velocity;
        actual_velocity_pub_->publish(actual_msg);

        std_msgs::msg::Float64 error_msg;
        error_msg.data = velocity_error;
        velocity_error_pub_->publish(error_msg);

        std_msgs::msg::Float64 torque_msg;
        torque_msg.data = control_torque;
        control_torque_pub_->publish(torque_msg);
    }

    // =========================
    // RMCS 输入
    // =========================

    InputInterface<Eigen::Vector2d>
        joystick_right_;

    InputInterface<double>
        measurement_;

    // =========================
    // RMCS 输出
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
    // 速度滤波
    // =========================

    rmcs_core::filter::LowPassFilter<1>
        velocity_filter_;

    // =========================
    // Foxglove Publisher
    // =========================

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_velocity_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        actual_velocity_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        velocity_error_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        control_torque_pub_;

    int debug_publish_counter_ = 0;
};

}  // namespace rmcs_core::controller::motor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor::M3508PidController,
    rmcs_executor::Component)