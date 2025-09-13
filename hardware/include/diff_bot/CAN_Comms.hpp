#ifndef DIFF_BOT_CAN_COMMS_HPP
#define DIFF_BOT_CAN_COMMS_HPP

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>
#include <cstring>
#include <vector>
#include <rclcpp/rclcpp.hpp>

class CANComms
{
public:
    CANComms(const rclcpp::Logger &logger, const std::string &interface_name = "can0")
        : logger_(logger)
    {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
        {
            RCLCPP_ERROR(logger_, "Socket creation failed: %s", strerror(errno));
            return;
        }

        struct ifreq ifr;
        std::strncpy(ifr.ifr_name, interface_name.c_str(), IFNAMSIZ);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
        {
            RCLCPP_ERROR(logger_, "Getting interface index failed: %s", strerror(errno));
            close(sock_);
            sock_ = -1;
            return;
        }

        addr_.can_family = AF_CAN;
        addr_.can_ifindex = ifr.ifr_ifindex;

        if (bind(sock_, (struct sockaddr *)&addr_, sizeof(addr_)) < 0)
        {
            RCLCPP_ERROR(logger_, "Binding CAN socket failed: %s", strerror(errno));
            close(sock_);
            sock_ = -1;
            return;
        }
    }

    ~CANComms() {   if (sock_ >= 0) {   close(sock_);   }   }

    bool connect(const std::string &interface, int bitrate) {
        std::string cmd = 
            "sudo /sbin/ip link set " + interface + " down && " +
            "sudo /sbin/ip link set " + interface + " type can bitrate " + std::to_string(bitrate) + " && " +
            "sudo /sbin/ip link set " + interface + " up";
        int ret = std::system(cmd.c_str());

        RCLCPP_INFO(logger_, "Command : '%s'", cmd.c_str());

        if (WIFEXITED(ret)) {
            int exit_code = WEXITSTATUS(ret);
            RCLCPP_INFO(logger_, "System returned '%d'", exit_code);
        }
        return (ret == 0);
    }

    bool disconnect(const std::string &interface) {
        std::string cmd = "ip link set " + interface + " down";
        int ret = std::system(cmd.c_str());
        return (ret == 0);
    }

    bool send_frame(uint32_t can_id, std::vector<uint8_t> data)
    {
        if (sock_ < 0)
        {
            RCLCPP_ERROR(logger_, "CAN socket is not initialized");
            return false;
        }

        if (data.size() > 6)
        {
            RCLCPP_WARN(logger_, "Data too long to append CRC (%zu bytes)", data.size());
            return false;
        }

        uint16_t crc_value = crc(data);
        data.push_back((crc_value >> 8) & 0xFF);
        data.push_back(crc_value & 0xFF);

        struct can_frame frame{};
        frame.can_id = can_id;
        frame.can_dlc = data.size();
        std::copy(data.begin(), data.end(), frame.data);

        ssize_t nbytes = write(sock_, &frame, sizeof(frame));
        if (nbytes != sizeof(frame))
        {
            RCLCPP_ERROR(logger_, "Failed to send CAN frame: %s", strerror(errno));
            return false;
        }

        return true;
    }

    bool receive_frame(struct can_frame &received_frame, int timeout_ms = 100)
    {
        if (sock_ < 0)
        {
            RCLCPP_ERROR(logger_, "CAN socket is not initialized");
            return false;
        }

        fd_set read_set;
        struct timeval timeout;
        FD_ZERO(&read_set);
        FD_SET(sock_, &read_set);
        timeout.tv_sec = 0;
        timeout.tv_usec = timeout_ms * 1000;

        int ret = select(sock_ + 1, &read_set, nullptr, nullptr, &timeout);
        if (ret > 0 && FD_ISSET(sock_, &read_set))
        {
            ssize_t nbytes = read(sock_, &received_frame, sizeof(received_frame));
            if (nbytes != sizeof(received_frame))
            {
                RCLCPP_ERROR(logger_, "[CAN_Comms]:Incomplete CAN frame read: %zd bytes", nbytes);
                return false;
            }

            if (received_frame.can_dlc < 3)
            {
                RCLCPP_WARN(logger_, "[CAN_Comms]: Received frame too short to contain CRC (DLC: %u)", received_frame.can_dlc);
                return false;
            }

            std::vector<uint8_t> payload(received_frame.data, received_frame.data + received_frame.can_dlc - 2);
            uint16_t received_crc = (received_frame.data[received_frame.can_dlc - 2] << 8) |
                                     received_frame.data[received_frame.can_dlc - 1];
            uint16_t computed_crc = crc(payload);

            if (received_crc != computed_crc)
            {
                RCLCPP_WARN(logger_, "[CAN_Comms]: CRC mismatch: received=0x%04X, computed=0x%04X", received_crc, computed_crc);
                return false;
            }

            return true;
        }

        RCLCPP_ERROR(logger_, "[CAN_Comms]: timeout reached! No message sent");
        return false; // timeout or error
    }

private:
    int sock_{-1};
    struct sockaddr_can addr_{};
    rclcpp::Logger logger_;

    uint16_t crc(const std::vector<uint8_t> &data)
    {
        const uint16_t polynomial = 0x4599;
        uint16_t crc = 0;

        for (size_t byte_idx = 0; byte_idx < data.size(); ++byte_idx)
        {
            uint8_t current_byte = data[byte_idx];
            for (int i = 7; i >= 0; --i)
            {
                bool bit = (current_byte >> i) & 1;
                bool crc_msb = (crc >> 14) & 1;
                crc <<= 1;
                crc |= bit;
                if (crc_msb != bit)
                {
                    crc ^= polynomial;
                }
                crc &= 0x7FFF;
            }
        }
        return crc;
    }
};

#endif // DIFF_BOT_CAN_COMMS_HPP
