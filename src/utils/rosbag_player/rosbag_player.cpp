#include <thread>
#include <fstream>
#include <stdexcept>
#include <algorithm>
#include <cctype>
#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <rosbag2_cpp/reader.hpp>
#include <rosbag2_cpp/readers/sequential_reader.hpp>
#include <rosbag2_storage/serialized_bag_message.hpp>
#include <sensor_msgs/msg/compressed_image.hpp>
#include <sensor_msgs/msg/detail/image__struct.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <vision_interface/msg/match_info.hpp>
#include <rosbag2_storage/storage_options.hpp>     
#include <rosbag2_cpp/converter_options.hpp>

using namespace std::chrono_literals;

namespace
{
bool has_suffix(const std::string & value, const std::string & suffix)
{
    return value.size() >= suffix.size() &&
           value.compare(value.size() - suffix.size(), suffix.size(), suffix) == 0;
}

std::string trim(std::string value)
{
    value.erase(value.begin(), std::find_if(value.begin(), value.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
    value.erase(std::find_if(value.rbegin(), value.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), value.end());
    return value;
}

std::string detect_storage_id_from_metadata(const std::string & bag_path)
{
    std::ifstream metadata_file(bag_path + "/metadata.yaml");
    if (!metadata_file.is_open()) {
        return "";
    }

    std::string line;
    const std::string key = "storage_identifier:";
    while (std::getline(metadata_file, line)) {
        const auto pos = line.find(key);
        if (pos == std::string::npos) {
            continue;
        }
        return trim(line.substr(pos + key.size()));
    }

    return "";
}

std::string normalize_topic_name(std::string topic_name)
{
    if (!topic_name.empty() && topic_name.front() != '/') {
        topic_name = "/" + topic_name;
    }
    return topic_name;
}

bool is_compressed_image_topic(std::string topic_name)
{
    topic_name = normalize_topic_name(std::move(topic_name));
    return topic_name == "/compressed_image" ||
           topic_name == "/camera_image/compressed";
}

bool is_raw_image_topic(std::string topic_name)
{
    topic_name = normalize_topic_name(std::move(topic_name));
    return topic_name == "/camera_image";
}
}  // namespace

void on_exit([[maybe_unused]] int sig)
{
    RCUTILS_LOG_INFO("Exit by Ctrl+C");
    rclcpp::shutdown();
    exit(0);
}

class RosbagPlayer : public rclcpp::Node {
public:
    RosbagPlayer(const rclcpp::NodeOptions& options)
        : Node("rosbag_player_node", options)
    {
        this->declare_parameter<std::string>("rosbag_file", "");
        this->get_parameter("rosbag_file", rosbag_file);

        if (rosbag_file.empty()) {
            throw std::runtime_error("Parameter 'rosbag_file' must not be empty.");
        }

        pointcloud_publisher_ =
            this->create_publisher<sensor_msgs::msg::PointCloud2>(
                "/livox/lidar", 10);
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            "camera_image", rclcpp::SensorDataQoS());
        match_info_publisher_ =
            this->create_publisher<vision_interface::msg::MatchInfo>(
                "/match_info", 10);
        signal(SIGINT, on_exit);
        //reader_.open(rosbag_file);
        open_bag();
        processing_thread_ =
            std::make_shared<std::thread>(&RosbagPlayer::play_bag, this);
    }

    ~RosbagPlayer()
    {
        if (processing_thread_ && processing_thread_->joinable()) {
            processing_thread_->join();
        }
    }

private:

    void open_bag()
    {
        storage_options_ = {};
        storage_options_.uri = rosbag_file;

        // 根据文件后缀判断格式
        if (has_suffix(rosbag_file, ".mcap")) {
            storage_options_.storage_id = "mcap";
            RCLCPP_INFO(this->get_logger(), "Detected MCAP format. Opening: %s", rosbag_file.c_str());
        } 
        else if (has_suffix(rosbag_file, ".db3")) {
            storage_options_.storage_id = "sqlite3";
            RCLCPP_INFO(this->get_logger(), "Detected SQLite3 format. Opening: %s", rosbag_file.c_str());
        } 
        else {
            storage_options_.storage_id = detect_storage_id_from_metadata(rosbag_file);
            if (!storage_options_.storage_id.empty()) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "Detected %s bag from metadata. Opening: %s",
                    storage_options_.storage_id.c_str(),
                    rosbag_file.c_str());
            } else {
                // 如果无法识别，尝试置空，让 rosbag2 自行处理
                RCLCPP_INFO(this->get_logger(), "Opening directory or unknown format: %s", rosbag_file.c_str());
            }
        }

        converter_options_ = {};
        
        // 使用配置好的选项打开 reader
        reader_.open(storage_options_, converter_options_);
    }

    void reopen_bag()
    {
        reader_.close();
        reader_.open(storage_options_, converter_options_);
    }

    void play_bag()
    {
        while (rclcpp::ok()) {
            if (!reader_.has_next()) {
                RCLCPP_INFO(this->get_logger(), "Reached end of bag, restarting playback.");
                reopen_bag();
                if (!reader_.has_next()) {
                    RCLCPP_WARN(this->get_logger(), "Bag is empty after reopening: %s", rosbag_file.c_str());
                    std::this_thread::sleep_for(100ms);
                    continue;
                }
            }

            auto start_time = std::chrono::high_resolution_clock::now();
            auto bag_message = reader_.read_next();
            auto ros_time = rclcpp::Clock().now();
            const auto topic_name = normalize_topic_name(bag_message->topic_name);

            if (topic_name == "/livox/lidar") {
                auto pointcloud_msg =
                    std::make_shared<sensor_msgs::msg::PointCloud2>();
                rclcpp::Serialization<sensor_msgs::msg::PointCloud2>
                                          serialization;
                rclcpp::SerializedMessage serialized_msg(
                    *bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg,
                                                  pointcloud_msg.get());
                pointcloud_msg->header.stamp = ros_time;
                pointcloud_publisher_->publish(*pointcloud_msg);
            }
            else if (is_compressed_image_topic(topic_name)) {
                auto image_msg =
                    std::make_shared<sensor_msgs::msg::CompressedImage>();
                rclcpp::Serialization<sensor_msgs::msg::CompressedImage>
                                          serialization;
                rclcpp::SerializedMessage serialized_msg(
                    *bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg,
                                                  image_msg.get());
                auto img = cv::imdecode(image_msg->data, cv::IMREAD_COLOR);
                auto msg =
                    cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", img)
                        .toImageMsg();
                msg->header.stamp = ros_time;
                image_publisher_->publish(*msg);
            }
            else if (is_raw_image_topic(topic_name)) {
                auto image_msg =
                    std::make_shared<sensor_msgs::msg::Image>();
                rclcpp::Serialization<sensor_msgs::msg::Image>
                                          serialization;
                rclcpp::SerializedMessage serialized_msg(
                    *bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg,
                                                  image_msg.get());
                image_msg->header.stamp = ros_time;
                image_publisher_->publish(*image_msg);
            }
            else if (topic_name == "/match_info") {
                auto match_info_msg =
                    std::make_shared<vision_interface::msg::MatchInfo>();
                rclcpp::Serialization<vision_interface::msg::MatchInfo>
                                          serialization;
                rclcpp::SerializedMessage serialized_msg(
                    *bag_message->serialized_data);
                serialization.deserialize_message(&serialized_msg,
                                                  match_info_msg.get());
                match_info_publisher_->publish(*match_info_msg);
            }

            auto end_time = std::chrono::high_resolution_clock::now();
            auto duration =
                std::chrono::duration_cast<std::chrono::milliseconds>(
                    end_time - start_time)
                    .count();
            if ((duration < 100) && (duration > 1)) {
                std::this_thread::sleep_for(
                    100ms - std::chrono::milliseconds(duration));
            }
        }
        RCLCPP_INFO(this->get_logger(), "No more messages in the bag.");
        rclcpp::shutdown();
    }

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr
        pointcloud_publisher_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<vision_interface::msg::MatchInfo>::SharedPtr
                                 match_info_publisher_;
    rosbag2_cpp::Reader          reader_;
    rosbag2_storage::StorageOptions storage_options_;
    rosbag2_cpp::ConverterOptions converter_options_;
    std::shared_ptr<std::thread> processing_thread_;
    std::string                  rosbag_file;
};

RCLCPP_COMPONENTS_REGISTER_NODE(RosbagPlayer)
