#ifndef DIFF_BOT_STEPPER_SYSTEM_HARDWARE_HPP_
#define DIFF_BOT_STEPPER_SYSTEM_HARDWARE_HPP_

#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/logger.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "diff_bot/visibility_control.h"
#include "diff_bot/CAN_Comms.hpp"
#include "diff_bot/Stepper.hpp"

namespace diff_bot
{
    class StepperSystemHardware : public hardware_interface::SystemInterface
    {

    struct Config {
    
        std::vector<std::string> joint_names;
        std::vector<double> joint_reductions;
        std::vector<std::string> CAN_id;

        std::string interface_name = "";
        double CAN_id[2] = {0x00, 0x00};
        int CAN_rate = 0;
        int timout_ms = 0;
    };

    public:
        RCLCPP_SHARED_PTR_DEFINITIONS(StepperSystemHardware)

        DIFF_BOT_PUBLIC
        hardware_interface::CallbackReturn on_init(
            const hardware_interface::HardwareInfo & info) override;

        DIFF_BOT_PUBLIC
        hardware_interface::CallbackReturn on_configure(
            const rclcpp_lifecycle::State & previous_state);

        // DIFF_BOT_PUBLIC
        // hardware_interface::CallbackReturn on_cleanup(
        //     const rclcpp_lifecycle::State & previous_state);

        DIFF_BOT_PUBLIC
        std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

        DIFF_BOT_PUBLIC
        std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

        DIFF_BOT_PUBLIC
        hardware_interface::CallbackReturn on_activate(
            const rclcpp_lifecycle::State & previous_state) override;

        DIFF_BOT_PUBLIC
        hardware_interface::CallbackReturn on_deactivate(
            const rclcpp_lifecycle::State & previous_state) override;

        DIFF_BOT_PUBLIC
        hardware_interface::return_type read(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

        DIFF_BOT_PUBLIC
        hardware_interface::return_type write(
            const rclcpp::Time & time, const rclcpp::Duration & period) override;

    private:

        Config cfg_;
        rclcpp::Logger logger_ = rclcpp::get_logger("stepper_system_hardware");  
        CANComms comms_{logger_}; 

        std::unique_ptr<StepperMotor> motor_L, motor_R;
        std::string cmd_mode_ = "";

        double joint_commands[2][3];
        double joint_states[2][2];
    };
}

#endif  // DIFF_BOT_STEPPER_SYSTEM_HARDWARE_HPP_