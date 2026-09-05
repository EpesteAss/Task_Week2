#include <memory>

#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>
#include <rmcs_msgs/switch.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"
#include "hardware/device/dr16.hpp"
#include "hardware/device/remote_control.hpp"

namespace rmcs_core::hardware {

class M3508TestHardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {

public:
    M3508TestHardware()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)}
        , command_component_{
              create_partner_component<CommandTransmitter>(
                  get_component_name() + "_command", *this)}
        , motor_{*this, *command_component_, "/m3508"}
        , remote_control_{std::make_unique<device::RemoteControl>(*this)} {

        // =========================================================
        // 参数
        // =========================================================

        max_velocity_ =
            get_parameter("max_velocity").as_double();

        // =========================================================
        // M3508
        // 独立注册
        // CAN ID = 3
        // =========================================================

        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kM3508,
                3}
                .set_reduction_ratio(1.0));

        // =========================================================
        // DR16
        // =========================================================

        remote_control_->register_dr16(&dr16_);

        // =========================================================
        // 目标速度
        //
        // DR16 右摇杆 Y
        //      ↓
        // /m3508/target_velocity
        //
        // 这个接口提供给官方 PidController 的 setpoint
        // =========================================================

        register_output(
            "/m3508/target_velocity",
            target_velocity_,
            0.0);

        // =========================================================
        // C Board
        // VID = 0xA11C
        // PID = 0xD401
        // =========================================================

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string());
    }

    // =============================================================
    // RMCS 主周期
    // 1000 Hz
    // =============================================================

    void update() override {

        // ---------------------------------------------------------
        // 1. 更新 M3508 反馈
        // ---------------------------------------------------------

        motor_.update_status();

        // ---------------------------------------------------------
        // 2. 更新 DR16
        // ---------------------------------------------------------

        dr16_.update_status();
        remote_control_->update();

        // ---------------------------------------------------------
        // 3. DR16 → 目标速度
        //
        // joystick_right().y() ∈ [-1, 1]
        //
        // 例如：
        // max_velocity = 100
        //
        // y =  1.0 → +100
        // y =  0.5 →  +50
        // y =  0.0 →    0
        // y = -0.5 →  -50
        // y = -1.0 → -100
        // ---------------------------------------------------------

        *target_velocity_ =
            dr16_.joystick_right().y() * max_velocity_;
    }

private:

    // =============================================================
    // 命令发送组件
    // =============================================================

    class CommandTransmitter
        : public rmcs_executor::Component {

    public:
        explicit CommandTransmitter(
            M3508TestHardware& hardware)
            : hardware_(hardware) {}

        void update() override {
            hardware_.command_update();
        }

    private:
        M3508TestHardware& hardware_;
    };

    // =============================================================
    // CAN 命令发送
    // =============================================================

    void command_update() {

        const bool emergency_stop =
            dr16_.switch_left() == rmcs_msgs::Switch::DOWN
            && dr16_.switch_right() == rmcs_msgs::Switch::DOWN;

        auto builder = board_->start_transmit();

        // =========================================================
        // 双下急停
        // =========================================================

        if (emergency_stop) {

            if (!last_emergency_stop_) {
                RCLCPP_WARN(
                    get_logger(),
                    "EMERGENCY STOP: SW1 and SW2 are both DOWN");
            }

            last_emergency_stop_ = true;

            builder.can_transmit(
                Spec::kCans.kCan1,
                {
                    .can_id = motor_.send_id(),
                    .can_data =
                        device::CanPacket8{
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::PaddingQuarter{},
                            device::CanPacket8::Quarter{0},
                            device::CanPacket8::PaddingQuarter{}}
                            .as_bytes(),
                });

            return;
        }

        last_emergency_stop_ = false;

        // =========================================================
        // 正常发送
        //
        // 官方 PidController 的输出已经进入：
        // /m3508/control_torque
        //
        // DjiMotor 内部会读取并生成 CAN command
        // =========================================================

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = motor_.send_id(),
                .can_data =
                    device::CanPacket8{
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{}}
                        .as_bytes(),
            });
    }

    // =============================================================
    // CAN 接收
    // =============================================================

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data) override {

        if (data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() < 8)
            return;

        // 我们使用 C Board CAN1
        if (can != Spec::kCans.kCan1)
            return;

        // =========================================================
        // M3508 ID = 3
        //
        // Feedback CAN ID = 0x200 + 3 = 0x203
        // =========================================================

        if (data.can_id == 0x203) {
            motor_.store_status(data.can_data);
        }
    }

    // =============================================================
    // UART / DR16 接收
    // =============================================================

    void uart_receive_callback(
        const Spec::Uart& uart,
        const View::Uart& data) override {

        if (uart == Spec::kUarts.kDbus) {
            dr16_.store_status(
                data.uart_data.data(),
                data.uart_data.size());
        }
    }

    // =============================================================
    // 成员变量
    // =============================================================

    std::shared_ptr<CommandTransmitter>
        command_component_;

    std::unique_ptr<librmcs::board::CBoard>
        board_;

    device::DjiMotor
        motor_;

    device::Dr16
        dr16_;

    std::unique_ptr<device::RemoteControl>
        remote_control_;

    // =============================================================
    // RMCS 输出
    // =============================================================

    OutputInterface<double>
        target_velocity_;

    // =============================================================
    // 参数
    // =============================================================

    double max_velocity_ = 100.0;

    // =============================================================
    // 急停状态
    // =============================================================

    bool last_emergency_stop_ = false;
};

}  // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::M3508TestHardware,
    rmcs_executor::Component)