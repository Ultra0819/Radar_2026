#include "rm_serial_driver.hpp"
#include "crc_table.hpp"
namespace rm_serial {

RMSerialDriver::RMSerialDriver(const rclcpp::NodeOptions& options)
    : rclcpp::Node("rm_serial_driver", options) 
{
    // 1. 声明并获取参数
    this->declare_parameter<std::string>("serial_port", "/dev/ttyUSB0");
    this->declare_parameter<int>("force_self_color", -1);
    
    std::string port_name;
    this->get_parameter("serial_port", port_name);
    this->get_parameter("force_self_color", self_color_);



    if (self_color_ != -1) {
        RCLCPP_WARN(this->get_logger(), "【测试模式】已强制指定己方阵营为: %s", self_color_ == 0 ? "蓝方" : "红方");
    } else {
        RCLCPP_INFO(this->get_logger(), "【实战模式】等待裁判系统 0x0201 数据包分配阵营...");
    }

    // 2. 初始化串口 (使用参数传入的设备号)
    init_serial(port_name);

    // 3. 初始化 ROS 发布和订阅
    match_info_pub_ = this->create_publisher<vision_interface::msg::MatchInfo>("/match_info", 10);
    auto_pub_ = this->create_publisher<vision_interface::msg::Auto>("/auto_info", 10);
    kalman_sub_ = this->create_subscription<vision_interface::msg::DetectResult>(
        "/robust_detect", 10, std::bind(&RMSerialDriver::kalman_callback, this, std::placeholders::_1));
    trigger_sub_ = this->create_subscription<std_msgs::msg::UInt8>(
    "/trigger_vulnerability", 10, std::bind(&RMSerialDriver::trigger_callback, this, std::placeholders::_1));    
    // 4. 启动接收线程 (读取裁判系统)
    receive_thread_ = std::thread(&RMSerialDriver::receive_data_loop, this);

    // 5. 定时器 5Hz 发送数据包
    send_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(200), // 5Hz = 200ms
        std::bind(&RMSerialDriver::send_map_data, this));

    RCLCPP_INFO(this->get_logger(), "RM Serial Driver Started.");
}

RMSerialDriver::~RMSerialDriver() {
    if (receive_thread_.joinable()) {
        receive_thread_.join();
    }
    if (serial_fd_ > 0) {
        close(serial_fd_);
    }
}

void RMSerialDriver::init_serial(const std::string& port_name) {
    serial_fd_ = open(port_name.c_str(), O_RDWR | O_NOCTTY | O_NDELAY);
    if (serial_fd_ == -1) {
        RCLCPP_ERROR(this->get_logger(), "无法打开串口 %s", port_name.c_str());
        return;
    }
    struct termios options;
    tcgetattr(serial_fd_, &options);
    cfmakeraw(&options); // 【关键修复】：设置为原始模式，关闭终端特殊字符处理
    cfsetispeed(&options, B115200); // 裁判系统波特率 115200
    cfsetospeed(&options, B115200);
    options.c_cflag |= (CLOCAL | CREAD);
    options.c_cflag &= ~PARENB; // 无校验位
    options.c_cflag &= ~CSTOPB; // 1位停止位
    options.c_cflag &= ~CSIZE;
    options.c_cflag |= CS8;     // 8位数据位
    tcsetattr(serial_fd_, TCSANOW, &options);
}

void RMSerialDriver::kalman_callback(const vision_interface::msg::DetectResult::SharedPtr msg) {
    std::lock_guard<std::mutex> lock(detect_mutex_);
    latest_detect_msg_ = *msg;
}

// 封装发送逻辑 (命令码 0x0305: 雷达地图数据)
void RMSerialDriver::send_map_data() {
    MapRobotData map_data = {}; // 初始化全0

    vision_interface::msg::DetectResult detect_data;
    {
        std::lock_guard<std::mutex> lock(detect_mutex_);
        detect_data = latest_detect_msg_;
    }

    // Helper 宏：将 float(米) 转为 uint16_t(厘米)。若为0则保持0（协议规定全0表示未识别）
    auto to_cm = [](float coord) -> uint16_t {
        if (coord <= 0.0f) return 0;
        return static_cast<uint16_t>(coord * 100.0f);
    };

    // 2026 协议的 0x0305 需要同时发送敌我双方坐标。
    // 索引顺序：0-英雄, 1-工程, 2-步兵3, 3-步兵4, 4-空中, 5-哨兵
    const float* ally_x  = (self_color_ == 0) ? detect_data.blue_x.data() : detect_data.red_x.data();
    const float* ally_y  = (self_color_ == 0) ? detect_data.blue_y.data() : detect_data.red_y.data();
    const float* oppo_x  = (self_color_ == 0) ? detect_data.red_x.data()  : detect_data.blue_x.data();
    const float* oppo_y  = (self_color_ == 0) ? detect_data.red_y.data()  : detect_data.blue_y.data();

    map_data.oppo_hero_x = to_cm(oppo_x[0]);         map_data.oppo_hero_y = to_cm(oppo_y[0]);
    map_data.oppo_engineer_x = to_cm(oppo_x[1]);     map_data.oppo_engineer_y = to_cm(oppo_y[1]);
    map_data.oppo_inf3_x = to_cm(oppo_x[2]);         map_data.oppo_inf3_y = to_cm(oppo_y[2]);
    map_data.oppo_inf4_x = to_cm(oppo_x[3]);         map_data.oppo_inf4_y = to_cm(oppo_y[3]);
    map_data.oppo_aerial_x = to_cm(oppo_x[4]);       map_data.oppo_aerial_y = to_cm(oppo_y[4]);
    map_data.oppo_sentry_x = to_cm(oppo_x[5]);       map_data.oppo_sentry_y = to_cm(oppo_y[5]);

    map_data.ally_hero_x = to_cm(ally_x[0]);         map_data.ally_hero_y = to_cm(ally_y[0]);
    map_data.ally_engineer_x = to_cm(ally_x[1]);     map_data.ally_engineer_y = to_cm(ally_y[1]);
    map_data.ally_inf3_x = to_cm(ally_x[2]);         map_data.ally_inf3_y = to_cm(ally_y[2]);
    map_data.ally_inf4_x = to_cm(ally_x[3]);         map_data.ally_inf4_y = to_cm(ally_y[3]);
    map_data.ally_aerial_x = to_cm(ally_x[4]);       map_data.ally_aerial_y = to_cm(ally_y[4]);
    map_data.ally_sentry_x = to_cm(ally_x[5]);       map_data.ally_sentry_y = to_cm(ally_y[5]);

    send_packet(0x0305, reinterpret_cast<uint8_t*>(&map_data), sizeof(map_data));
}

void RMSerialDriver::send_packet(uint16_t cmd_id, const uint8_t* data, uint16_t len) {

    std::vector<uint8_t> packet;
    FrameHeader header;
    header.data_length = len;
    header.seq = seq_++;
    
    // 计算包头 CRC8
    header.crc8 = get_crc8(reinterpret_cast<uint8_t*>(&header), 4);
    
    // 组装包头
    uint8_t* header_ptr = reinterpret_cast<uint8_t*>(&header);
    packet.insert(packet.end(), header_ptr, header_ptr + sizeof(FrameHeader));
    
    // 组装 CmdID (小端)
    packet.push_back(cmd_id & 0xFF);
    packet.push_back((cmd_id >> 8) & 0xFF);
    
    // 组装数据
    packet.insert(packet.end(), data, data + len);
    
    // 计算整包 CRC16
    uint16_t crc16 = get_crc16(packet.data(), packet.size());
    packet.push_back(crc16 & 0xFF);
    packet.push_back((crc16 >> 8) & 0xFF);
    
    // 在 write(serial_fd_, packet.data(), packet.size()); 之前打印出来看：
    std::string hex_str;
    for(auto b : packet) {
    char buf[5];
    sprintf(buf, "%02X ", b);
    hex_str += buf;
    }
    //RCLCPP_INFO(this->get_logger(), "准备发送的数据包: %s", hex_str.c_str());

    // 发送
    // 真正发送时再判断串口是否有效
    if (serial_fd_ > 0) {
        write(serial_fd_, packet.data(), packet.size());
    } 
}

// ======================== 接收与解包线程 ========================
void RMSerialDriver::receive_data_loop() {
    uint8_t buffer[1024];
    std::vector<uint8_t> rx_buffer;

    while (rclcpp::ok()) {
        int bytes_read = read(serial_fd_, buffer, sizeof(buffer));
        if (bytes_read > 0) {
            rx_buffer.insert(rx_buffer.end(), buffer, buffer + bytes_read);
            
            // 简单的状态机寻帧
            while (rx_buffer.size() >= 9) { // 帧头(5) + cmd_id(2) + 帧尾(2) = 9
                if (rx_buffer[0] != 0xA5) {
                    rx_buffer.erase(rx_buffer.begin());
                    continue;
                }
                
                // 【关键修复1】：先校验包头 5 个字节的 CRC8
                if (get_crc8(rx_buffer.data(), 4) != rx_buffer[4]) {
                    rx_buffer.erase(rx_buffer.begin()); // 校验失败，丢弃首字节
                    continue;
                }

                uint16_t data_length = rx_buffer[1] | (rx_buffer[2] << 8);

                // 【关键修复2】：防止异常包长导致死等，裁判系统单包不会超过256
                if(data_length > 256) {
                    rx_buffer.erase(rx_buffer.begin());
                    continue;
                }

                uint32_t frame_length = 5 + 2 + data_length + 2; // SOF+ID+DATA+CRC16
                
                if (rx_buffer.size() < frame_length) break; // 数据包不完整，等待下次读取
                
                // 校验 CRC8
                if (get_crc8(rx_buffer.data(), 4) == rx_buffer[4]) {
                    // 校验 CRC16
                    if (get_crc16(rx_buffer.data(), frame_length - 2) == 
                        (rx_buffer[frame_length - 2] | (rx_buffer[frame_length - 1] << 8))) 
                    {
                        // 校验通过，切片并解析
                        std::vector<uint8_t> packet(rx_buffer.begin(), rx_buffer.begin() + frame_length);
                        parse_packet(packet);
                        rx_buffer.erase(rx_buffer.begin(), rx_buffer.begin() + frame_length);
                        continue;
                    }
                }
                // 校验失败，丢弃包头
                rx_buffer.erase(rx_buffer.begin());
            }
        }
        usleep(5000); // 5ms 阻塞
    }
}

void RMSerialDriver::parse_packet(const std::vector<uint8_t>& packet) {
    uint16_t cmd_id = packet[5] | (packet[6] << 8);
    uint16_t data_length = packet[1] | (packet[2] << 8);
    const uint8_t* data_ptr = packet.data() + 7;

    //vision_interface::msg::MatchInfo match_info_msg;
    bool publish_flag = false;

    if (cmd_id == 0x0201) { 
        // 机器人状态数据，用于判断阵营 (参考协议附录: 9 是红方雷达，109 是蓝方雷达)
        uint8_t robot_id = data_ptr[0];
        if (robot_id == 9) self_color_ = 2;       // 红方
        else if (robot_id == 109) self_color_ = 0;// 蓝方
        
        this->match_info_msg.self_color = self_color_;
        publish_flag = true;
    } 
    else if (cmd_id == 0x0001) {
        // 比赛状态数据
        uint8_t game_progress = (data_ptr[0] >> 4) & 0x0F;
        uint16_t stage_remain_time = data_ptr[1] | (data_ptr[2] << 8);
        
        if (game_progress == 4) {
            // 比赛进行中：正数，代表剩余秒数
            this->match_info_msg.match_time = stage_remain_time;
        } else if (game_progress == 3) {
            // 5秒倒计时：负数，代表还差几秒开赛 (例如 -5, -4, -3...)
            match_info_msg.match_time = -stage_remain_time;   
        } 
        else if (game_progress == 5) {
            // 结算：-300
            match_info_msg.match_time = -300;
        }
        else {
            // 其他未开始阶段 (准备、自检等)：统一为 -100
            
            match_info_msg.match_time = -100;
        }
        if (game_progress != 4) {
            auto_msg_ = vision_interface::msg::Auto();
        }
        this->match_info_msg.self_color = self_color_; 
        publish_flag = true;
    }
    else if (cmd_id == 0x020E) {
        if (data_length < 1) {
            RCLCPP_WARN(this->get_logger(), "0x020E 数据长度异常: %u", static_cast<unsigned>(data_length));
            return;
        }

        auto_msg_.vulnerability_chance = data_ptr[0] & 0x03;          // bit 0-1
        auto_msg_.enemy_vulnerable_status = (data_ptr[0] >> 2) & 0x01; // bit 2
        
        auto_pub_->publish(auto_msg_); // 直接发布，不需要经过下面的 publish_flag 判断
        // 注意：这里不要设置 publish_flag = true; 否则会触发空 match_info 的发布
    }
    else if (cmd_id == 0x0105) {
        if (data_length < 3) {
            RCLCPP_WARN(this->get_logger(), "0x0105 数据长度异常: %u", static_cast<unsigned>(data_length));
            return;
        }

        // 偏移 1 的 uint16_t: bit 0-2 为最近一次己方飞镖击中的目标。
        const uint16_t dart_status =
            static_cast<uint16_t>(data_ptr[1]) |
            (static_cast<uint16_t>(data_ptr[2]) << 8);

        auto_msg_.dart_info = dart_status & 0x0007;

        auto_pub_->publish(auto_msg_);
    }
    else if (cmd_id == 0x0209) {
        if (data_length < 5) {
            RCLCPP_WARN(this->get_logger(), "0x0209 数据长度异常: %u", static_cast<unsigned>(data_length));
            return;
        }

        const uint32_t rfid_status =
            static_cast<uint32_t>(data_ptr[0]) |
            (static_cast<uint32_t>(data_ptr[1]) << 8) |
            (static_cast<uint32_t>(data_ptr[2]) << 16) |
            (static_cast<uint32_t>(data_ptr[3]) << 24);

        // bit0: 对方中央高地下方(bit11), bit1: 对方中央高地上方(bit12)。
        auto_msg_.hight_road =
            ((rfid_status >> 11) & 0x01) |
            (((rfid_status >> 12) & 0x01) << 1);
        auto_msg_.enemy_buff = (rfid_status >> 24) & 0x01;

        auto_pub_->publish(auto_msg_);
    }

    if (publish_flag) {
        match_info_pub_->publish(match_info_msg);
    }
}

void RMSerialDriver::trigger_callback(const std_msgs::msg::UInt8::SharedPtr msg) {
    std::vector<uint8_t> data;
    
    // 1. 构建交互数据包头部 (子ID 0x0121)
    RobotInteractionDataHeader interaction_header;
    interaction_header.data_cmd_id = 0x0121;
    // 蓝方雷达ID为109，红方雷达ID为9
    interaction_header.sender_id = (self_color_ == 0) ? 109 : 9; 
    interaction_header.receiver_id = 0x8080; // 0x8080代表裁判系统服务器

    // 2. 构建雷达指令段
    RadarCmdData radar_cmd;
    memset(&radar_cmd, 0, sizeof(radar_cmd));
    radar_cmd.radar_cmd = msg->data; // 填入单调递增的序列号

    // 3. 内存拼接
    uint8_t* header_ptr = reinterpret_cast<uint8_t*>(&interaction_header);
    data.insert(data.end(), header_ptr, header_ptr + sizeof(RobotInteractionDataHeader));
    
    uint8_t* cmd_ptr = reinterpret_cast<uint8_t*>(&radar_cmd);
    data.insert(data.end(), cmd_ptr, cmd_ptr + sizeof(RadarCmdData));

    // 4. 发送 0x0301 协议
    send_packet(0x0301, data.data(), data.size());
    RCLCPP_INFO(this->get_logger(), "串口已通过 0x0301 下发易伤指令!");
}

// ---------------- CRC 实现补充 ---------------- 
uint8_t RMSerialDriver::get_crc8(const uint8_t *pchMessage, uint32_t dwLength) {
    uint8_t ucCRC8 = CRC8_INIT;
    while (dwLength--) {
        ucCRC8 = CRC8_TAB[ucCRC8 ^ (*pchMessage++)];
    }
    return ucCRC8;
}

uint16_t RMSerialDriver::get_crc16(const uint8_t *pchMessage, uint32_t dwLength) {
    uint16_t wCRC = CRC16_INIT;
    while (dwLength--) {
        wCRC = ((wCRC >> 8) & 0xFF) ^ wCRC_Table[(wCRC ^ (*pchMessage++)) & 0xFF];
    }
    return wCRC;
}

} // namespace rm_serial

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rm_serial::RMSerialDriver>(rclcpp::NodeOptions());
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
