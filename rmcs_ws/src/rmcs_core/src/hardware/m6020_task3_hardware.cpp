#include <memory>

#include <librmcs/board/c_board.hpp>
#include <rclcpp/node.hpp>
#include <rmcs_executor/component.hpp>

#include "hardware/device/can_packet.hpp"
#include "hardware/device/dji_motor.hpp"

namespace rmcs_core::hardware {

class M6020Task3Hardware
    : public rmcs_executor::Component
    , public rclcpp::Node
    , public librmcs::board::CBoard::Callback {
public:
    M6020Task3Hardware()
        : Node{
              get_component_name(),
              rclcpp::NodeOptions{}
                  .automatically_declare_parameters_from_overrides(true)}
        , command_component_{
              create_partner_component<CommandTransmitter>(
                  get_component_name() + "_command", *this)}
        , motor_{*this, *command_component_, "/m6020"} {

        // GM6020, CAN ID = 1
        motor_.configure(
            device::DjiMotor::Config{
                device::DjiMotor::Type::kGM6020,
                1}
                .enable_multi_turn_angle());

        board_ = std::make_unique<librmcs::board::CBoard>(
            *this,
            get_parameter("board_serial").as_string());
    }

    void update() override {
        motor_.update_status();
    }

private:
    class CommandTransmitter : public rmcs_executor::Component {
    public:
        explicit CommandTransmitter(M6020Task3Hardware& hardware)
            : hardware_(hardware) {}

        void update() override {
            hardware_.command_update();
        }

    private:
        M6020Task3Hardware& hardware_;
    };

    void command_update() {
        auto builder = board_->start_transmit();

        builder.can_transmit(
            Spec::kCans.kCan1,
            {
                .can_id = motor_.send_id(),
                .can_data =
                    device::CanPacket8{
                        motor_.generate_command(),
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{},
                        device::CanPacket8::PaddingQuarter{}}
                        .as_bytes(),
            });
    }

    void can_receive_callback(
        const Spec::Can& can,
        const View::Can& data) override {

        if (data.is_extended_can_id
            || data.is_remote_transmission
            || data.can_data.size() < 8)
            return;

        if (can != Spec::kCans.kCan1)
            return;

        // GM6020 ID = 1 -> feedback CAN ID = 0x205
        if (data.can_id == 0x205)
            motor_.store_status(data.can_data);
    }

    std::shared_ptr<CommandTransmitter> command_component_;

    std::unique_ptr<librmcs::board::CBoard> board_;

    device::DjiMotor motor_;
};

}  // namespace rmcs_core::hardware

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
    rmcs_core::hardware::M6020Task3Hardware,
    rmcs_executor::Component)