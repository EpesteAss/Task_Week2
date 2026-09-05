#include <atomic>
#include <cmath>
#include <cstdint>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::debug {

class Task3TargetAngle
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Task3TargetAngle()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)} {

        // =========================================================
        // YAML 默认目标角度
        // =========================================================

        target_angle_value_.store(
            get_parameter("target_angle").as_double());

        // =========================================================
        // RMCS 输出
        //
        // Task3TargetAngle
        //        ↓
        // /m6020/target_angle
        //        ↓
        // 外环 PID
        // =========================================================

        register_output(
            "/m6020/target_angle",
            target_angle_,
            target_angle_value_.load());

        // =========================================================
        // 只读取 Hardware 提供的反馈
        //
        // 注意：
        // 绝对不要在这里读取 /m6020/target_velocity
        // 或 /m6020/control_torque
        //
        // 否则会形成循环依赖。
        // =========================================================

        register_input(
            "/m6020/angle",
            actual_angle_);

        register_input(
            "/m6020/velocity",
            actual_velocity_);

        // =========================================================
        // ROS2：目标角度输入
        // =========================================================

        target_angle_subscription_ =
            create_subscription<std_msgs::msg::Float64>(
                "/task3/target_angle",
                rclcpp::QoS(10),
                [this](
                    const std_msgs::msg::Float64::SharedPtr msg) {

                    if (std::isfinite(msg->data)) {
                        target_angle_value_.store(msg->data);
                    }
                });

        // =========================================================
        // ROS2：实际角度
        // =========================================================

        actual_angle_publisher_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task3/actual_angle",
                10);

        // =========================================================
        // ROS2：实际速度
        // =========================================================

        actual_velocity_publisher_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task3/actual_velocity",
                10);

        // =========================================================
        // ROS2：当前目标角度
        // =========================================================

        target_angle_publisher_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task3/target_angle_feedback",
                10);
    }

    void update() override {

        // =========================================================
        // 目标角度写入 RMCS
        // =========================================================

        const double target_angle =
            target_angle_value_.load();

        *target_angle_ = target_angle;

        // =========================================================
        // RMCS 1000 Hz
        // ROS2 100 Hz
        // =========================================================

        ++counter_;

        if (counter_ < 10)
            return;

        counter_ = 0;

        // =========================================================
        // 实际角度
        // =========================================================

        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = *actual_angle_;

        actual_angle_publisher_->publish(
            angle_msg);

        // =========================================================
        // 实际速度
        // =========================================================

        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = *actual_velocity_;

        actual_velocity_publisher_->publish(
            velocity_msg);

        // =========================================================
        // 当前目标角度
        // =========================================================

        std_msgs::msg::Float64 target_msg;
        target_msg.data = target_angle;

        target_angle_publisher_->publish(
            target_msg);
    }

private:

    // =============================================================
    // RMCS
    // =============================================================

    OutputInterface<double>
        target_angle_;

    InputInterface<double>
        actual_angle_;

    InputInterface<double>
        actual_velocity_;

    // =============================================================
    // ROS2 → RMCS
    // =============================================================

    std::atomic<double>
        target_angle_value_{0.0};

    // =============================================================
    // ROS2
    // =============================================================

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_angle_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        actual_angle_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        actual_velocity_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_angle_publisher_;

    // =============================================================
    // 1000 Hz → 100 Hz
    // =============================================================

    std::uint32_t counter_ = 0;
};

}  // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::debug::Task3TargetAngle,
    rmcs_executor::Component)
