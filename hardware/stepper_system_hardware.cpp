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

        // Extract joint names, reductions, and CAN_id from hardware parameters
        for (const auto &joint : info_.joints) {
            cfg_.joint_names.push_back(joint.name);
            cfg_.joint_reductions.push_back(std::stod(joint.parameters.at("reduction")));   
            cfg_.CAN_id.push_back(static_cast<uint16_t>
                (std::stoul(joint.parameters.at("CAN_id"), nullptr, 16)));
        }

        // Extract global hardware parameters
        cfg_.interface_name = info_.hardware_parameters["interface_name"];
        cfg_.CAN_rate = std::stoi(info_.hardware_parameters["CAN_rate"]);
        cfg_.timout_ms = std::stoi(info_.hardware_parameters["timeout"]);
        cfg_.loop_rate = std::stoi(info_.hardware_parameters["loop_rate"]);

        // Initialize state and command storage
        joint_states.resize(cfg_.joint_names.size(), {0.0, 0.0});
        joint_cmds.resize(cfg_.joint_names.size(), {0.0, 0.0, 0.0});

        RCLCPP_INFO(logger_, "info passed successfully!");

        // Validate command and state interfaces for each joint
        // assign a command mode (position or speed)
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
                setCommandMode(SPEED);
                RCLCPP_INFO(logger_, "Joint '%s' is in speed mode", joint_name.c_str());
            }
            else if (cmds.size() == 3 &&
                    cmds[0].name == hardware_interface::HW_IF_POSITION &&
                    cmds[1].name == hardware_interface::HW_IF_VELOCITY &&
                    cmds[2].name == hardware_interface::HW_IF_ACCELERATION)
            {
                setCommandMode(POSITION);
                RCLCPP_INFO(logger_, "Joint '%s' is in position mode", joint_name.c_str());
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

            RCLCPP_INFO(logger_, "Joint '%s' has commnand interface : ", joint_name.c_str());
            for (auto &cmd : cmds) {
                RCLCPP_INFO(logger_, "  - %s", cmd.name.c_str());
            }
            RCLCPP_INFO(logger_, "Joint '%s' has state interface : ", joint_name.c_str());
            for (auto &state : states) {
                RCLCPP_INFO(logger_, "  - %s", state.name.c_str());
            }
        }
        
        RCLCPP_INFO(logger_, "All joints have valid command and state interfaces");

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_configure(
            const rclcpp_lifecycle::State & /*previous_state*/) {

        RCLCPP_INFO(logger_, "Configuring %zu stepper motors...",
                    cfg_.joint_names.size());

        for(size_t i = 0; i < cfg_.joint_names.size(); i++) {
            motors.push_back(std::make_unique<StepperMotor>(
                cfg_.joint_names[i], 
                cfg_.joint_reductions[i], 
                cfg_.CAN_id[i], 
                logger_));

            RCLCPP_INFO(logger_, "Joint '%s': reduction=%.2f, CAN ID=0x%X",
                        cfg_.joint_names[i].c_str(),
                        cfg_.joint_reductions[i],
                        cfg_.CAN_id[i]);
        }

        return hardware_interface::CallbackReturn::SUCCESS;
    }

    hardware_interface::CallbackReturn StepperSystemHardware::on_activate(
        const rclcpp_lifecycle::State & /*previous_state*/) {
        
        RCLCPP_INFO(logger_, "Activating CAN device: %s, bitrate: %d", cfg_.interface_name.c_str(), cfg_.CAN_rate);

        while(!comms_.connect(cfg_.interface_name, cfg_.CAN_rate)) {

            RCLCPP_WARN(logger_, "CAN device not connected, retrying ...");
            rclcpp::sleep_for(std::chrono::milliseconds(200));
        }

        RCLCPP_INFO(logger_, "Attaching motors to CAN bus...");

        for (const auto& motor : motors) {
            if (motor->attachComms(comms_)) {
                RCLCPP_ERROR(logger_, "Failed to attach motor %s to CAN bus", motor->name_.c_str());
                return hardware_interface::CallbackReturn::ERROR;
            }
        }
        RCLCPP_INFO(logger_, "Successfully attached motors to CAN bus");
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

        for (size_t i = 0; i < cfg_.joint_names.size(); i++) {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_POSITION, &joint_states[i][0]));
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_VELOCITY, &joint_states[i][1]));
        }

      return state_interfaces;
    }

    std::vector<hardware_interface::CommandInterface> StepperSystemHardware::export_command_interfaces() {
        
        std::vector<hardware_interface::CommandInterface> command_interfaces;

        for (size_t i = 0; i < cfg_.joint_names.size(); i++) {

            // If in position mode, export position interfaces first
            if (cmd_mode_ == CommandMode::POSITION) {
                command_interfaces.emplace_back(hardware_interface::CommandInterface(
                    cfg_.joint_names[i], hardware_interface::HW_IF_POSITION, &joint_cmds[i][0]));
            }

            // Always export velocity + acceleration
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_VELOCITY, &joint_cmds[i][1]));
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                cfg_.joint_names[i], hardware_interface::HW_IF_ACCELERATION, &joint_cmds[i][2]));
        }

        return command_interfaces;
    }

    hardware_interface::return_type StepperSystemHardware::read(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) 
    {   
        // update motor positions and velocities
        for (size_t i = 0; i < motors.size(); i++) {
            motors[i]->pos = motors[i]->read_encoder_position();
            motors[i]->vel = motors[i]->read_speed();
          
            // update joint states
            joint_states[i][0] = motors[i]->pos / motors[i]->gear_reduction;
            joint_states[i][1] = motors[i]->vel / motors[i]->gear_reduction;
        }
        updateDiffJointStates();

        return hardware_interface::return_type::OK;
    }

    // differential gearbox mechanism
    void StepperSystemHardware::updateDiffJointStates() {

        // Use last two motors
        auto& motor_L = motors[motors.size() - 2];
        auto& motor_R = motors[motors.size() - 1];
        auto& carrier_state = joint_states[joint_states.size() - 2];
        auto& bevel_state = joint_states[joint_states.size() - 1];

        carrier_state[0] = (motor_L->pos / motor_L->gear_reduction + motor_R->pos / motor_R->gear_reduction) / 2;
        carrier_state[1] = (motor_L->vel / motor_L->gear_reduction + motor_R->vel / motor_R->gear_reduction) / 2;

        bevel_state[0] = (motor_L->pos / motor_L->gear_reduction - motor_R->pos / motor_R->gear_reduction) / 2;
        bevel_state[1] = (motor_L->vel / motor_L->gear_reduction - motor_R->vel / motor_R->gear_reduction) / 2;
    }

    hardware_interface::return_type StepperSystemHardware::write(
            const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/) {
        
        if (cmd_mode_ == CommandMode::POSITION) {
            // parse through all joints and send position commands
            for(size_t i = 0; i < motors.size(); i++) {

                double motor_cmds[3] = {0, 0, 0};
                // convert joint commands to motor commands
                for(size_t j = 0; j < std::size(motor_cmds); j++) {
                    if (i < motors.size() - 2) {
                        motor_cmds[j] = motors[i]->gear_reduction * joint_cmds[i][j];
                    } else {
                        motor_cmds[j] = updateDiffMotorCmd(i,j);
                    }
                }  

                // send motor position command
                if(!motors[i]->send_absolute_position(
                    static_cast<int32_t>(motor_cmds[0]),
                    static_cast<uint16_t>(motor_cmds[1]),
                    static_cast<uint8_t>(motor_cmds[2])))
                {
                    RCLCPP_ERROR(logger_, "Failed to send position command to motor %s", motors[i]->name_.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            } 
        } else {
            // send speed commands to all motors
            for(size_t i = 0; i < motors.size(); i++) {

                double motor_cmds[2] = {0, 0};
                // convert joint commands to motor commands
                for(size_t j = 0; j < std::size(motor_cmds); j++) {
                    if (i < motors.size() - 2) {
                        motor_cmds[j] = motors[i]->gear_reduction * joint_cmds[i][j];
                    } else {
                        motor_cmds[j] = updateDiffMotorCmd(i,j+1);
                    }
                } 

                // send motor speed command 
                if(!motors[i]->send_speed(
                    static_cast<uint16_t>(motor_cmds[0]),
                    static_cast<uint8_t>(motor_cmds[1])))
                {
                    RCLCPP_ERROR(logger_, "Failed to send speed command to motor %s", motors[i]->name_.c_str());
                    return hardware_interface::return_type::ERROR;
                }
            }
        }

        return hardware_interface::return_type::OK;
    }

    double StepperSystemHardware::updateDiffMotorCmd(size_t motor_index, size_t cmd_index) {

        // Use last two motors
        auto& motor_L = motors[motors.size() - 2];
        auto& motor_R = motors[motors.size() - 1];
        auto& carrier_cmd = joint_cmds[joint_cmds.size() - 2];
        auto& bevel_cmd = joint_cmds[joint_cmds.size() - 1];

        return (motor_index == motors.size() - 2) ? 
            (carrier_cmd[cmd_index] + bevel_cmd[cmd_index]) * motor_L->gear_reduction : 
            (carrier_cmd[cmd_index] - bevel_cmd[cmd_index]) * motor_R->gear_reduction;
    }

    hardware_interface::return_type StepperSystemHardware::prepare_command_mode_switch(
        const std::vector<std::string>& start_ifaces,
        const std::vector<std::string>& /*stop_ifaces*/)
    {
        
        // If nothing is being started, nothing to validate or change.
        if (start_ifaces.empty()) {
            return hardware_interface::return_type::OK;
        }

        const size_t n_joints = cfg_.joint_names.size();

        // Count how many of each interface are being started.
        size_t n_pos = 0, n_vel = 0, n_acc = 0, n_other = 0;

        for (const auto& full : start_ifaces) {
            // Interface names look like "joint_name/interface"
            const auto slash = full.rfind('/');
            std::string_view ifname =
                (slash == std::string::npos)
                    ? std::string_view{full}
                    : std::string_view{full}.substr(slash + 1);

            if (ifname == hardware_interface::HW_IF_POSITION) {
            ++n_pos;
            } else if (ifname == hardware_interface::HW_IF_VELOCITY) {
            ++n_vel;
            } else if (ifname == hardware_interface::HW_IF_ACCELERATION) {
            ++n_acc;
            } else {
            ++n_other; // unknown interface
            }
        }

        if (n_other > 0) {
            RCLCPP_ERROR(logger_, "[prepare_command_mode_switch] unsupported interfaces requested");
            return hardware_interface::return_type::ERROR;
        }

        if (n_pos > 0) {
            
            // Position mode requires (per joint): position + velocity + acceleration
            if (n_pos != n_joints || n_vel != n_joints || n_acc != n_joints) {
            RCLCPP_ERROR(logger_,
                        "[prepare_command_mode_switch] position mode requires starting "
                        "position, velocity, acceleration for ALL joints "
                        "Recieved : pos=%zu vel=%zu acc=%zu, joints=%zu)",
                        n_pos, n_vel, n_acc, n_joints);
            return hardware_interface::return_type::ERROR;
            }
            requested_cmd_mode_ = CommandMode::POSITION;

        } else {
            // Speed mode requires (per joint): velocity + acceleration
            if (n_vel != n_joints || n_acc != n_joints) {
            RCLCPP_ERROR(logger_,
                        "[prepare_command_mode_switch] speed mode requires starting "
                        "velocity, acceleration for ALL joints "
                        "Recieved : vel=%zu acc=%zu, joints=%zu)",
                        n_vel, n_acc, n_joints);
            return hardware_interface::return_type::ERROR;
            }
            requested_cmd_mode_ = CommandMode::SPEED;
        }
        
        return hardware_interface::return_type::OK;
    }

    hardware_interface::return_type StepperSystemHardware::perform_command_mode_switch(
        const std::vector<std::string>& start_ifaces,
        const std::vector<std::string>& stop_ifaces) {

        if (requested_cmd_mode_ == cmd_mode_) {
            // Already in this mode, nothing to do
            RCLCPP_INFO(logger_, "[perform_command_mode_switch] already in requested mode");
            return hardware_interface::return_type::OK;
        }

        for (const auto& iface : start_ifaces) {
            RCLCPP_INFO(logger_, "[prepare_command_mode_switch] Starting interface: %s", iface.c_str());
        }

        for (const auto& iface : stop_ifaces) {
            RCLCPP_INFO(logger_, "[prepare_command_mode_switch] Stopping interface: %s", iface.c_str());
        }

        setCommandMode(requested_cmd_mode_);
        RCLCPP_INFO(logger_, "[perform_command_mode_switch] switched to %s mode",
                    (cmd_mode_ == CommandMode::POSITION) ? "position" : "speed");
        
        for (auto& command: joint_cmds) {
            command[0] = 0.0; // position
            command[1] = 0.0; // velocity
            command[2] = 0.0; // acceleration
        }

        RCLCPP_INFO(logger_, "[perform_command_mode_switch]: Reintialized all joint commands to zero!");
        return hardware_interface::return_type::OK;
    }

} // namespace robotic_arm

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  diff_bot::StepperSystemHardware, hardware_interface::SystemInterface)