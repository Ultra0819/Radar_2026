#include <chrono>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "opencv2/opencv.hpp"

using namespace std::chrono_literals;

class VideoPublisherNode : public rclcpp::Node
{
public:
    VideoPublisherNode() : Node("video_publisher_node")
    {
        // 声明参数
        this->declare_parameter<std::string>("video_path", "/home/hwx/视频/video_save/5.22.mp4");
        this->declare_parameter<double>("fps", 30.0);
        this->declare_parameter<bool>("loop", true);

        // 获取参数
        std::string video_path = this->get_parameter("video_path").as_string();
        double fps = this->get_parameter("fps").as_double();
        loop_ = this->get_parameter("loop").as_bool();

        if (video_path.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Please provide a valid video_path parameter!");
            rclcpp::shutdown();
            return;
        }

        // 打开视频文件
        cap_.open(video_path);
        if (!cap_.isOpened()) {
            RCLCPP_ERROR(this->get_logger(), "Failed to open video file: %s", video_path.c_str());
            rclcpp::shutdown();
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Successfully opened video: %s", video_path.c_str());
        RCLCPP_INFO(this->get_logger(), "Publishing at %.1f FPS. Loop mode: %s", fps, loop_ ? "ON" : "OFF");

        // 创建发布者，由于你的 detect 节点使用的是 SensorDataQoS，这里保持兼容
        pub_ = this->create_publisher<sensor_msgs::msg::Image>("camera_image", rclcpp::SensorDataQoS());

        // 根据设定的 FPS 计算定时器周期
        auto timer_period = std::chrono::duration<double>(1.0 / fps);
        timer_ = this->create_wall_timer(
            timer_period, std::bind(&VideoPublisherNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        cv::Mat frame;
        cap_ >> frame; // 读取下一帧

        // 如果视频播放结束
        if (frame.empty()) {
            if (loop_) {
                RCLCPP_INFO(this->get_logger(), "Video ended. Restarting loop...");
                cap_.set(cv::CAP_PROP_POS_FRAMES, 0); // 回到视频开头
                cap_ >> frame;
            } else {
                RCLCPP_INFO(this->get_logger(), "Video ended. Shutting down.");
                rclcpp::shutdown();
                return;
            }
        }

        // 将 OpenCV 的 Mat (BGR格式) 转换为 ROS 2 的 Image 消息
        // 你的 detect 节点中使用的是 cv_bridge::toCvCopy(msg, "bgr8")，所以这里统一使用 "bgr8" 发送
        auto img_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        
        // 填入时间戳和坐标系
        img_msg->header.stamp = this->now();
        img_msg->header.frame_id = "camera_optical_frame";

        pub_->publish(*img_msg);
    }

    cv::VideoCapture cap_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_;
    rclcpp::TimerBase::SharedPtr timer_;
    bool loop_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VideoPublisherNode>());
    rclcpp::shutdown();
    return 0;
}