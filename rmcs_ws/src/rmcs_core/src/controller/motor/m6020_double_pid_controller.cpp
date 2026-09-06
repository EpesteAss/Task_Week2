#include <atomic>
#include <algorithm>
#include <cmath>
#include <memory>

#include <eigen3/Eigen/Core>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <std_msgs/msg/float64.hpp>

namespace rmcs_core::controller::motor {

//=========================
//该代码仅在调试初期使用过，最终用的rmcs组件

class M6020DoublePidController
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    M6020DoublePidController()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)} {

        // =========================
        // RMCS 接口
        // =========================

        // 实际角度反馈
        register_input(
            "/m6020/angle",
            angle_);

        // 实际速度反馈
        register_input(
            "/m6020/velocity",
            velocity_);

        // 输出给 6020 的控制扭矩
        register_output(
            "/m6020/control_torque",
            control_,
            0.0);

        // =========================
        // PID 参数
        // =========================

        angle_kp_ = get_parameter("angle_kp").as_double();
        angle_ki_ = get_parameter("angle_ki").as_double();
        angle_kd_ = get_parameter("angle_kd").as_double();

        velocity_kp_ = get_parameter("velocity_kp").as_double();
        velocity_ki_ = get_parameter("velocity_ki").as_double();
        velocity_kd_ = get_parameter("velocity_kd").as_double();

        max_velocity_ =
            get_parameter("max_velocity").as_double();

        max_torque_ =
            get_parameter("max_torque").as_double();

        angle_integral_min_ =
            get_parameter("angle_integral_min").as_double();

        angle_integral_max_ =
            get_parameter("angle_integral_max").as_double();

        velocity_integral_min_ =
            get_parameter("velocity_integral_min").as_double();

        velocity_integral_max_ =
            get_parameter("velocity_integral_max").as_double();

        // ROS2 目标角度
        target_angle_subscription_ =
            create_subscription<std_msgs::msg::Float64>(
                "/m6020/target_angle",
                rclcpp::QoS(10),
                [this](const std_msgs::msg::Float64::SharedPtr msg) {
                    target_angle_.store(msg->data);
                });
    }

    void update() override {

        constexpr double dt = 0.001;  // RMCS 1000 Hz

        const double current_angle = *angle_;
        const double current_velocity = *velocity_;
        const double target_angle = target_angle_.load();

        if (!std::isfinite(current_angle)
            || !std::isfinite(current_velocity)
            || !std::isfinite(target_angle)) {

            reset_pid();

            *control_ = 0.0;
            return;
        }

        // =====================================================
        // 外环：角度 PID
        // =====================================================

        // 角度是周期量，误差限制到 [-pi, pi)
        // 保证尽可能走最短方向
        double angle_error =
            std::remainder(
                target_angle - current_angle,
                2.0 * M_PI);

        const double angle_p =
            angle_kp_ * angle_error;

        angle_integral_ += angle_error * dt;

        angle_integral_ =
            std::clamp(
                angle_integral_,
                angle_integral_min_,
                angle_integral_max_);

        const double angle_i =
            angle_ki_ * angle_integral_;

        double angle_d = 0.0;

        if (has_last_angle_error_) {
            angle_d =
                angle_kd_
                * (angle_error - last_angle_error_)
                / dt;
        }

        last_angle_error_ = angle_error;
        has_last_angle_error_ = true;

        // 外环输出：目标速度
        double target_velocity =
            angle_p + angle_i + angle_d;

        target_velocity =
            std::clamp(
                target_velocity,
                -max_velocity_,
                max_velocity_);

        // =====================================================
        // 内环：速度 PID
        // =====================================================

        const double velocity_error =
            target_velocity - current_velocity;

        const double velocity_p =
            velocity_kp_ * velocity_error;

        velocity_integral_ += velocity_error * dt;

        velocity_integral_ =
            std::clamp(
                velocity_integral_,
                velocity_integral_min_,
                velocity_integral_max_);

        const double velocity_i =
            velocity_ki_ * velocity_integral_;

        double velocity_d = 0.0;

        if (has_last_velocity_error_) {
            velocity_d =
                velocity_kd_
                * (velocity_error - last_velocity_error_)
                / dt;
        }

        last_velocity_error_ = velocity_error;
        has_last_velocity_error_ = true;

        // 内环输出：控制扭矩
        double output =
            velocity_p
            + velocity_i
            + velocity_d;

        output =
            std::clamp(
                output,
                -max_torque_,
                max_torque_);

        *control_ = output;
    }

private:
    void reset_pid() {
        angle_integral_ = 0.0;
        velocity_integral_ = 0.0;

        last_angle_error_ = 0.0;
        last_velocity_error_ = 0.0;

        has_last_angle_error_ = false;
        has_last_velocity_error_ = false;
    }

    // =========================
    // RMCS 输入
    // =========================

    InputInterface<double> angle_;
    InputInterface<double> velocity_;

    // =========================
    // RMCS 输出
    // =========================

    OutputInterface<double> control_;

    // =========================
    // ROS2 目标角度
    // =========================

    std::atomic<double> target_angle_{0.0};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_angle_subscription_;

    // =========================
    // 外环角度 PID
    // =========================

    double angle_kp_ = 0.0;
    double angle_ki_ = 0.0;
    double angle_kd_ = 0.0;

    double angle_integral_ = 0.0;

    double angle_integral_min_ = -1.0;
    double angle_integral_max_ = 1.0;

    double last_angle_error_ = 0.0;
    bool has_last_angle_error_ = false;

    // =========================
    // 外环输出限制
    // =========================

    double max_velocity_ = 20.0;

    // =========================
    // 内环速度 PID
    // =========================

    double velocity_kp_ = 0.0;
    double velocity_ki_ = 0.0;
    double velocity_kd_ = 0.0;

    double velocity_integral_ = 0.0;

    double velocity_integral_min_ = -10.0;
    double velocity_integral_max_ = 10.0;

    double last_velocity_error_ = 0.0;
    bool has_last_velocity_error_ = false;

    // =========================
    // 扭矩限制
    // =========================

    double max_torque_ = 1.0;
};

}  // namespace rmcs_core::controller::motor

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::controller::motor::M6020DoublePidController,
    rmcs_executor::Component)