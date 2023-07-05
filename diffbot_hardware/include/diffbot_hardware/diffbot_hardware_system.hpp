#ifndef DIFFBOT_HARDWARE__DIFFBOT_HARDWARE_SYSTEM_HPP_
#define DIFFBOT_HARDWARE__DIFFBOT_HARDWARE_SYSTEM_HPP_


#include "hardware_interface/system_interface.hpp"


#include <memory>
#include <string>
#include <vector>

#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diffbot_hardware/opencr.hpp"
#include "diffbot_hardware/visibility_control.h"


namespace diffbot_hardware
{
    class DiffbotHardwareSystem : public hardware_interface::SystemInterface
    {
        public:
            RCLCPP_SHARED_PTR_DEFINITIONS(DiffbotHardwareSystem);

            DIFFBOT_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_init(
                const hardware_interface::HardwareInfo & info) override;

            DIFFBOT_HARDWARE_PUBLIC
            std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

            DIFFBOT_HARDWARE_PUBLIC
            std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

            DIFFBOT_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_activate(
                const rclcpp_lifecycle::State & previous_state) override;

            DIFFBOT_HARDWARE_PUBLIC
            hardware_interface::CallbackReturn on_deactivate(
                const rclcpp_lifecycle::State & previous_state) override;

            DIFFBOT_HARDWARE_PUBLIC
            hardware_interface::return_type read(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;

            DIFFBOT_HARDWARE_PUBLIC
            hardware_interface::return_type write(
                const rclcpp::Time & time, const rclcpp::Duration & period) override;
        
        private:
            uint8_t id_;
            std::string usb_port_;
            uint32_t baud_rate_;
            uint8_t heartbeat_;

            // std::array<int32_t, 4> joints_acceleration_;
            // std::array<int32_t, 4> joints_velocity_;

            std::unique_ptr<OpenCR> opencr_;

            std::vector<double> dxl_wheel_commands_;

            std::vector<double> dxl_positions_;
            std::vector<double> dxl_velocities_;

    };

}

#endif