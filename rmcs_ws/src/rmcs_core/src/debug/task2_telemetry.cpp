#include <cstdint>
#include <memory>

#include <rclcpp/node.hpp>
#include <std_msgs/msg/float64.hpp>

#include <rmcs_executor/component.hpp>

namespace rmcs_core::debug {

class Task2Telemetry
    : public rmcs_executor::Component
    , public rclcpp::Node {

public:
    Task2Telemetry()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)} {

        // =========================================================
        // RMCS 内部接口
        //
        // 注意：
        // 这些接口是在别的 Component 中已经存在的，
        // 这里仅注册 Input，不会重复创建。
        // =========================================================

        register_input(
            "/m3508/target_velocity",
            target_velocity_);

        register_input(
            "/m3508/velocity",
            actual_velocity_);

        register_input(
            "/m3508/control_torque",
            control_torque_);

        // =========================================================
        // ROS2 Topic
        // =========================================================

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

        // RMCS 调度是 1 kHz。
        //
        // 每 10 个周期发布一次：
        // 1000 Hz / 10 = 100 Hz
        //
        // PID 本身仍然保持 1 kHz，
        // 只是 Foxglove 遥测降到 100 Hz，
        // 避免刷屏。

        ++counter_;

        if (counter_ < 10)
            return;

        counter_ = 0;

        const double target =
            *target_velocity_;

        const double actual =
            *actual_velocity_;

        const double control =
            *control_torque_;

        const double error =
            target - actual;

        // =========================================================
        // target velocity
        // =========================================================

        std_msgs::msg::Float64 target_msg;
        target_msg.data = target;

        target_velocity_pub_->publish(target_msg);

        // =========================================================
        // actual velocity
        // =========================================================

        std_msgs::msg::Float64 actual_msg;
        actual_msg.data = actual;

        actual_velocity_pub_->publish(actual_msg);

        // =========================================================
        // velocity error
        // =========================================================

        std_msgs::msg::Float64 error_msg;
        error_msg.data = error;

        velocity_error_pub_->publish(error_msg);

        // =========================================================
        // control torque
        // =========================================================

        std_msgs::msg::Float64 control_msg;
        control_msg.data = control;

        control_torque_pub_->publish(control_msg);
    }

private:

    // =============================================================
    // RMCS Input Interface
    // =============================================================

    InputInterface<double>
        target_velocity_;

    InputInterface<double>
        actual_velocity_;

    InputInterface<double>
        control_torque_;

    // =============================================================
    // ROS2 Publisher
    // =============================================================

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_velocity_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        actual_velocity_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        velocity_error_pub_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        control_torque_pub_;

    // =============================================================
    // 1 kHz → 100 Hz
    // =============================================================

    std::uint32_t counter_ = 0;
};

}  // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::debug::Task2Telemetry,
    rmcs_executor::Component)
