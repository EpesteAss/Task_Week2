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
            target_angle_value_.load());

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
    }

    void update() override {

        const double current_angle = *actual_angle_;
        const double requested_target =
            target_angle_value_.load();

        
        constexpr double kPi =
            3.14159265358979323846;
        constexpr double kTwoPi =
            2.0 * kPi;

        
        double current_normalized =
            std::fmod(current_angle, kTwoPi);

        if (current_normalized < 0.0)
            current_normalized += kTwoPi;

        double target_normalized =
            std::fmod(requested_target, kTwoPi);

        if (target_normalized < 0.0)
            target_normalized += kTwoPi;

        
        double delta =
            target_normalized - current_normalized;

        if (delta > kPi)
            delta -= kTwoPi;

        if (delta < -kPi)
            delta += kTwoPi;

        
        if (delta > 0.0)
            delta -= kTwoPi;
        else if (delta < 0.0)
            delta += kTwoPi;

        
        const double target_angle =
            current_angle + delta;

        *target_angle_ = target_angle;

        ++counter_;

        if (counter_ < 10)
            return;

        counter_ = 0;

        std_msgs::msg::Float64 angle_msg;
        angle_msg.data = *actual_angle_;

        actual_angle_publisher_->publish(
            angle_msg);

        std_msgs::msg::Float64 velocity_msg;
        velocity_msg.data = *actual_velocity_;

        actual_velocity_publisher_->publish(
            velocity_msg);

        std_msgs::msg::Float64 target_msg;
        target_msg.data = target_angle;

        target_angle_publisher_->publish(
            target_msg);
    }

private:

    OutputInterface<double>
        target_angle_;

    InputInterface<double>
        actual_angle_;

    InputInterface<double>
        actual_velocity_;

    std::atomic<double>
        target_angle_value_{0.0};

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