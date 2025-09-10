#include "diff_bot/stepper_system_hardware.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace diff_bot {

    hardware_interface::CallbackReturn StepperSystemHardware::on_init(
        const hardware_interface::HardwareInfo & info) {
        
        if (
            hardware_interface::SystemInterface::on_init(info) !=
            hardware_interface::CallbackReturn::SUCCESS)
        {
            RCLCPP_ERROR(logger_, "info Failed!");
            return hardware_interface::CallbackReturn::ERROR;
        }

        cfg_.joint_names.reserve(info_.joints.size());
        cfg_.joint_reductions.reserve(info_.joints.size());

        // Extract joint names and reductions from hardware parameters
        for (const auto &joint : info_.joints) {
            cfg_.joint_names.push_back(joint.name);
        
            // reduction is expected to be provided as "<joint_name>_reduction" in hardware parameters
            auto reduction_key = joint.name + "_reduction";
            if (info_.hardware_parameters.count(reduction_key)) {
                cfg_.joint_reductions.push_back(std::stod(info_.hardware_parameters.at(reduction_key)));
            } else {
                RCLCPP_WARN(logger_, "No reduction specified for %s, defaulting to 1.0", joint.name.c_str());
                cfg_.joint_reductions.push_back(1.0f);
            }
        }

        cfg_.interface_name = info_.hardware_parameters["interface_name"];
        cfg_.CAN_rate = std::stoi(info_.hardware_parameters["CAN_rate"]);

        RCLCPP_INFO(logger_, "info passed successfully!");

        for (const hardware_interface::ComponentInfo & joint : info_.joints)
        {
          const auto & joint_name = joint.name;
          const auto & cmds = joint.command_interfaces;
          const auto & states = joint.state_interfaces;
              
          // Allow 2 or 3 command interfaces
          if (cmds.size() != 2 && cmds.size() != 3)
          {
              RCLCPP_FATAL(
                  logger_,"Joint '%s' has %zu command interfaces. Expected 2 (speed mode) or 3 (position mode).",
                  joint_name.c_str(), cmds.size());
              return hardware_interface::CallbackReturn::ERROR;
          }
        
          if (cmds.size() == 2 &&
              cmds[0].name == hardware_interface::HW_IF_VELOCITY &&
              cmds[1].name == hardware_interface::HW_IF_ACCELERATION)
          {
              cmd_mode_ = "speed";
              RCLCPP_INFO(logger_, "Hardware interface is in speed mode");
          }
          else if (cmds.size() == 3 &&
                   cmds[0].name == hardware_interface::HW_IF_POSITION &&
                   cmds[1].name == hardware_interface::HW_IF_VELOCITY &&
                   cmds[2].name == hardware_interface::HW_IF_ACCELERATION)
          {
              cmd_mode_ = "position";
              RCLCPP_INFO(logger_, "Hardware interface is in position mode");
          }
          else
          {
              RCLCPP_FATAL(
                  logger_, "Joint '%s' has unsupported command interface configuration.", joint_name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
        
          // Expect 2 state interfaces: position and velocity
          if (states.size() != 2 ||
              states[0].name != hardware_interface::HW_IF_POSITION ||
              states[1].name != hardware_interface::HW_IF_VELOCITY)
          {
              RCLCPP_FATAL(
                  logger_, "Joint '%s' must have exactly 2 state interfaces: 'position' and 'velocity'.", joint_name.c_str());
              return hardware_interface::CallbackReturn::ERROR;
          }
        }      

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {

        RCLCPP_INFO(logger_, "Configuring %zu stepper motors...",
                    cfg_.joint_names.size());
        
        motor_L = std::make_unique<StepperMotor>(cfg_.joint_names[0], cfg_.joint_reductions[0], cfg_.CAN_id[0], logger_);
        motor_R = std::make_unique<StepperMotor>(cfg_.joint_names[1], cfg_.joint_reductions[1], cfg_.CAN_id[1], logger_);

        RCLCPP_INFO(logger_, "Joint '%s' assigned CAN ID: 0x%X", cfg_.joint_names[0].c_str(), cfg_.CAN_id[0]);
        RCLCPP_INFO(logger_, "Joint '%s' assigned CAN ID: 0x%X", cfg_.joint_names[1].c_str(), cfg_.CAN_id[1]);

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        
        RCLCPP_INFO(logger_, "Activating CAN device: %s, bitrate: %d", cfg_.interface_name.c_str(), cfg_.CAN_rate);

        while(!comms_.connect(cfg_.interface_name, cfg_.CAN_rate)) {

            RCLCPP_WARN(logger_, "CAN device not connected, retrying ...");
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_INFO(logger_, "Successfully activated!");
        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_deactivate(
        const rclcpp_lifecycle::State & /*previous_state*/) {

      RCLCPP_INFO(logger_, "Deactivating hardware interface...");
      comms_.disconnect(cfg_.interface_name);
      RCLCPP_INFO(logger_, "Successfully deactivated!");

      return hardware_interface::CallbackReturn::SUCCESS;
    }
    
    std::vector<hardware_interface::StateInterface> StepperSystemHardware::export_state_interfaces() {
        
        std::vector<hardware_interface::StateInterface> state_interfaces;

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motor_L->name_, hardware_interface::HW_IF_POSITION, &joint_states[0][0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motor_L->name_, hardware_interface::HW_IF_VELOCITY, &joint_states[0][1]));

        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motor_R->name_, hardware_interface::HW_IF_POSITION, &joint_states[1][0]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            motor_R->name_, hardware_interface::HW_IF_VELOCITY, &joint_states[1][1]));

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> StepperSystemHardware::export_command_interfaces() {
        
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        // If in position mode, export position interfaces first
        if (cmd_mode_ == "position")
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                motor_L->name_, hardware_interface::HW_IF_POSITION, &joint_commands[0][0]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                motor_R->name_, hardware_interface::HW_IF_POSITION, &joint_commands[0][0]));
        }
        
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motor_L->name_, hardware_interface::HW_IF_VELOCITY, &joint_commands[0][1]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motor_L->name_, hardware_interface::HW_IF_ACCELERATION, &joint_commands[0][2]));

        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motor_R->name_, hardware_interface::HW_IF_VELOCITY, &joint_commands[1][1]));
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            motor_R->name_, hardware_interface::HW_IF_ACCELERATION, &joint_commands[1][2]));

        return command_interfaces;
    }

    hardware_interface::return_type StepperSystemHardware::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {   
        motor_L->pos = motor_L->read_encoder_position();
        motor_L->vel = motor_L->read_speed();
        motor_R->pos = motor_R->read_encoder_position();
        motor_R->vel = motor_R->read_speed();

        // differential gearbox mechanism
        joint_states[0][0] = (motor_L->pos / cfg_.joint_reductions[4] + motor_R->pos / cfg_.joint_reductions[4]) / 2;
        joint_states[0][1] = (motor_L->vel / cfg_.joint_reductions[4] + motor_R->vel / cfg_.joint_reductions[4]) / 2;

        joint_states[1][0] = (motor_L->pos / cfg_.joint_reductions[4] - motor_R->pos / cfg_.joint_reductions[4]) / 2;
        joint_states[1][1] = (motor_L->vel / cfg_.joint_reductions[4] - motor_R->vel / cfg_.joint_reductions[4]) / 2;

        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type StepperSystemHardware::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        
        if (cmd_mode_ == "position") {

            if(!motor_L->send_absolute_position(
                static_cast<int32_t>(motor_L->gear_reduction * (joint_commands[0][0] + joint_commands[1][0])),
                static_cast<uint16_t>(motor_L->gear_reduction * (joint_commands[0][1] + joint_commands[1][1])),
                static_cast<uint8_t>(motor_L->gear_reduction * (joint_commands[0][2] + joint_commands[1][2]))))
            {
                RCLCPP_ERROR(logger_, "Failed to send position command to motor %s", motor_L->name_.c_str());
                return hardware_interface::return_type::ERROR;
            }

            if(!motor_R->send_absolute_position(
                static_cast<int32_t>(motor_R->gear_reduction * (joint_commands[0][0] + joint_commands[1][0])),
                static_cast<uint16_t>(motor_R->gear_reduction * (joint_commands[0][1] + joint_commands[1][1])),
                static_cast<uint8_t>(motor_R->gear_reduction * (joint_commands[0][2] + joint_commands[1][2]))))
            {
                RCLCPP_ERROR(logger_, "Failed to send position command to motor %s", motor_R->name_.c_str());
                return hardware_interface::return_type::ERROR;
            }

        } else {

            if(!motor_L->send_speed(
                static_cast<uint16_t>(motor_L->gear_reduction * (joint_commands[0][1] + joint_commands[1][1])),
                static_cast<uint8_t>(motor_L->gear_reduction * (joint_commands[0][2] + joint_commands[1][2]))))
            {
                RCLCPP_ERROR(logger_, "Failed to send speed command to motor %s", motor_L->name_.c_str());
                return hardware_interface::return_type::ERROR;
            }

            if(!motor_R->send_speed(
                static_cast<uint16_t>(motor_R->gear_reduction * (joint_commands[0][1] + joint_commands[1][1])),
                static_cast<uint8_t>(motor_R->gear_reduction * (joint_commands[0][2] + joint_commands[1][2]))))
            {
                RCLCPP_ERROR(logger_, "Failed to send speed command to motor %s", motor_R->name_.c_str());
                return hardware_interface::return_type::ERROR;
            }
        }

        return hardware_interface::return_type::OK;
    }

} // namespace robotic_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_bot::StepperSystemHardware, hardware_interface::SystemInterface)