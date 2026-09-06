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

        target_angle_value_.store(
            get_parameter("target_angle").as_double());

        register_output(
            "/m6020/target_angle",
            target_angle_,
            0.0);

        register_input(
            "/m6020/angle",
            actual_angle_);

        register_input(
            "/m6020/velocity",
            actual_velocity_);

        target_angle_subscription_ =
            create_subscription<std_msgs::msg::Float64>(
                "/task3/target_angle",
                rclcpp::QoS(10),
                [this](
                    const std_msgs::msg::Float64::SharedPtr msg) {

                    if (std::isfinite(msg->data)) {
                        target_angle_value_.store(msg->data);
                        new_target_pending_.store(true);
                    }
                });

        actual_angle_publisher_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task3/actual_angle",
                10);

        actual_velocity_publisher_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task3/actual_velocity",
                10);

        target_angle_publisher_ =
            create_publisher<std_msgs::msg::Float64>(
                "/task3/target_angle_feedback",
                10);

        new_target_pending_.store(true);
    }

    void update() override {

        const double actual_angle = *actual_angle_;

        // 收到新的目标角度后，只计算一次对应的优弧目标。
        if (new_target_pending_.load()
            && std::isfinite(actual_angle)) {

            const double requested_target =
                target_angle_value_.load();

            target_angle_command_ =
                calculate_major_arc_target(
                    actual_angle,
                    requested_target);

            new_target_pending_.store(false);
        }

        *target_angle_ = target_angle_command_;

        ++counter_;

        if (counter_ < 10)
            return;

        counter_ = 0;

        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = actual_angle;

        actual_angle_publisher_->publish(
            angle_msg);

        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = *actual_velocity_;

        actual_velocity_publisher_->publish(
            velocity_msg);

        std_msgs::msg::Float64 target_msg;
        target_msg.data = target_angle_command_;

        target_angle_publisher_->publish(
            target_msg);
    }

private:

    static constexpr double kPi = 3.14159265358979323846;
    static constexpr double kTwoPi = 2.0 * kPi;

    // 把角度归一化到 [0, 2π)
    static double normalize_angle(double angle) {

        angle = std::fmod(angle, kTwoPi);

        if (angle < 0.0)
            angle += kTwoPi;

        return angle;
    }

    // 计算“优弧”的最终多圈目标角度。
    static double calculate_major_arc_target(
        double current_angle,
        double requested_target) {

        const double current_normalized =
            normalize_angle(current_angle);

        const double target_normalized =
            normalize_angle(requested_target);

        // 先得到 [-π, π] 的最短角度差
        double minor_delta =
            target_normalized - current_normalized;

        if (minor_delta > kPi)
            minor_delta -= kTwoPi;

        if (minor_delta < -kPi)
            minor_delta += kTwoPi;

        // 转换成另一条弧，也就是优弧
        double major_delta;

        if (minor_delta >= 0.0)
            major_delta = minor_delta - kTwoPi;
        else
            major_delta = minor_delta + kTwoPi;

        return current_angle + major_delta;
    }

    OutputInterface<double>
        target_angle_;

    InputInterface<double>
        actual_angle_;

    InputInterface<double>
        actual_velocity_;

    std::atomic<double>
        target_angle_value_{0.0};

    std::atomic<bool>
        new_target_pending_{true};

    double target_angle_command_{0.0};

    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr
        target_angle_subscription_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        actual_angle_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        actual_velocity_publisher_;

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr
        target_angle_publisher_;

    std::uint32_t counter_ = 0;
};

}  // namespace rmcs_core::debug

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::debug::Task3TargetAngle,
    rmcs_executor::Component)