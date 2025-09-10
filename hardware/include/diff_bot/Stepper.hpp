#ifndef DIFF_BOT_STEPPER_HPP
#define DIFF_BOT_STEPPER_HPP

#include "CAN_Comms.hpp"
#include <cstdint>
#include <string>
#include <rclcpp/rclcpp.hpp>

enum class CommandMode { SPEED, POSITION };

class StepperMotor
{
public:
    std::string name_;
    double gear_reduction = 1.0;
    double pos = 0;
    double vel = 0;
    double acc = 0;

    StepperMotor(std::string name, 
                 double reduction, 
                 uint16_t CAN_id,
                 const rclcpp::Logger &logger, 
                 )
        : name_(name), 
        gear_reduction(reduction),  
        CAN_id_(CAN_id)
        logger_(logger), {}

    bool attachComms(CANComms &comms) {
        comms_ = &comms;
        return comms_ != nullptr;
    }

    // Send command in speed mode (F6)
    bool send_speed(uint16_t rpm, uint8_t acc, bool clockwise = true) {
        
        if (rpm > 3000) rpm = 3000;

        uint8_t dir = clockwise ? 0x00 : 0x80; // bit7 = 1 for CCW, 0 for CW
        uint8_t high = ((rpm >> 8) & 0x0F);
        uint8_t low = rpm & 0xFF;
        uint8_t byte2 = dir | high;
        uint8_t byte3 = low;

        std::vector<uint8_t> payload = {0xF6, byte2, byte3, acc};
        return comms_->send_frame(CAN_id_, payload);
    }

    // Send command in position mode 4 (F5) - absolute motion by axis
    bool send_absolute_position(int32_t abs_axis, uint16_t rpm, uint8_t acc) {

        if (rpm > 3000) rpm = 3000;
        if (abs_axis < -8388607) abs_axis = -8388607;
        if (abs_axis > 8388607) abs_axis = 8388607;

        std::vector<uint8_t> payload = {
            0xF5,
            (uint8_t)((rpm >> 8) & 0xFF),
            (uint8_t)(rpm & 0xFF),
            acc,
            (uint8_t)(abs_axis & 0xFF),
            (uint8_t)((abs_axis >> 8) & 0xFF),
            (uint8_t)((abs_axis >> 16) & 0xFF)
        };

        return comms_->send_frame(CAN_id_, payload);
    }

    // Query speed (command 0x32)
    int16_t read_speed() {
        std::vector<uint8_t> query = {0x32};
        if (!comms_->send_frame(CAN_id_, query)) {
            RCLCPP_ERROR(logger_, "Failed to send speed query (0x32)");
            return 0;
        }

        struct can_frame response;
        if (comms_->receive_frame(response))
        {
            if (response.data[0] == 0x32)
            {
                int16_t speed = response.data[1] | (response.data[2] << 8);
                return speed;
            }
        }

        RCLCPP_ERROR(logger_, "Failed to receive speed response (0x32)");
        return 0;
    }

    // Query encoder position (command 0x31)
    int64_t read_encoder_position() {
        std::vector<uint8_t> query = {0x30};
        if (!comms_->send_frame(CAN_id_, query)) {
            RCLCPP_ERROR(logger_, "Failed to send encoder position query (0x30)");
            return 0;
        }

        struct can_frame response;
        if (comms_->receive_frame(response))
        {
            if (response.data[0] == 0x31)
            {
                int64_t pos = 0;
                for (int i = 5; i >= 1; --i)
                {
                    pos = (pos << 8) | response.data[i];
                }
                return pos;
            }
        }

        RCLCPP_ERROR(logger_, "Failed to receive encoder position (0x30)");
        return 0;
    }

    // Set microstepping using command 0x84
    bool set_microsteps(uint8_t microstep_value) {
        
        if (microstep_value == 0)
        {
            RCLCPP_WARN(logger_, "Invalid microstep value: 0");
            return false;
        }

        std::vector<uint8_t> payload = {0x84, microstep_value};
        return comms_->send_frame(CAN_id_, payload);
    }

private:

    CANComms* comms_ = nullptr;
    uint16_t CAN_id_;
    rclcpp::Logger logger_;
    CommandMode cmd_mode_;
};

#endif // DIFF_BOT_STEPPER_HPP
