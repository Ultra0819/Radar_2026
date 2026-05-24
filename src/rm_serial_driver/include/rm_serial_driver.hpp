#pragma once
#include <rclcpp/rclcpp.hpp>
#include <vision_interface/msg/detect_result.hpp>
#include <vision_interface/msg/match_info.hpp>
#include "vision_interface/msg/auto.hpp"
#include <std_msgs/msg/u_int8.hpp>
#include <termios.h>
#include <fcntl.h>
#include <unistd.h>
#include <thread>
#include <mutex>

namespace rm_serial {

// 强制1字节对齐，严格匹配官方协议
#pragma pack(push, 1)
struct FrameHeader {
    uint8_t sof = 0xA5;
    uint16_t data_length;
    uint8_t seq;
    uint8_t crc8;
};

// 0x0305: 选手端小地图接收雷达数据（2026 协议为敌我双方共 48 字节）
struct MapRobotData {
    uint16_t oppo_hero_x, oppo_hero_y;
    uint16_t oppo_engineer_x, oppo_engineer_y;
    uint16_t oppo_inf3_x, oppo_inf3_y;
    uint16_t oppo_inf4_x, oppo_inf4_y;
    uint16_t oppo_aerial_x, oppo_aerial_y;
    uint16_t oppo_sentry_x, oppo_sentry_y;

    uint16_t ally_hero_x, ally_hero_y;
    uint16_t ally_engineer_x, ally_engineer_y;
    uint16_t ally_inf3_x, ally_inf3_y;
    uint16_t ally_inf4_x, ally_inf4_y;
    uint16_t ally_aerial_x, ally_aerial_y;
    uint16_t ally_sentry_x, ally_sentry_y;
};

// 新增交互数据帧头 (0x0301)
struct RobotInteractionDataHeader {
    uint16_t data_cmd_id;
    uint16_t sender_id;
    uint16_t receiver_id;
};

// 新增雷达自主决策指令结构 (0x0121)
struct RadarCmdData {
    uint8_t radar_cmd;
    uint8_t password_cmd;
    uint8_t password[6];
};

#pragma pack(pop)

class RMSerialDriver : public rclcpp::Node {
public:
    RMSerialDriver(const rclcpp::NodeOptions& options);
    ~RMSerialDriver();

private:
    int serial_fd_;
    std::thread receive_thread_;
    uint8_t seq_ = 0;

    // ROS 通信接口
    rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr match_info_pub_;
    rclcpp::Publisher<vision_interface::msg::Auto>::SharedPtr auto_pub_;
    rclcpp::Subscription<vision_interface::msg::DetectResult>::SharedPtr kalman_sub_;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr trigger_sub_;
    rclcpp::TimerBase::SharedPtr send_timer_;

    // 数据缓存
    vision_interface::msg::DetectResult latest_detect_msg_;
    std::mutex detect_mutex_;
    int self_color_ = 2; // 0: 蓝方, 2: 红方
    vision_interface::msg::MatchInfo match_info_msg =[]{
        vision_interface::msg::MatchInfo msg;
        msg.match_time = -200; // 默认未连接
        return msg;
    }();
    vision_interface::msg::Auto auto_msg_;

    // 内部函数
    void init_serial(const std::string& port_name);
    void receive_data_loop();
    void parse_packet(const std::vector<uint8_t>& packet);
    void kalman_callback(const vision_interface::msg::DetectResult::SharedPtr msg);
    void send_map_data();
    void trigger_callback(const std_msgs::msg::UInt8::SharedPtr msg);
    // 协议封装与校验
    void send_packet(uint16_t cmd_id, const uint8_t* data, uint16_t len);
    uint8_t get_crc8(const uint8_t *pchMessage, uint32_t dwLength);
    uint16_t get_crc16(const uint8_t *pchMessage, uint32_t dwLength);
};

} // namespace rm_serial
