#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include "rclcpp/rclcpp.hpp"
#include <chrono>

bool isAllDigits(const std::string& s) {
    if (s.empty()) return false;
    for (char c : s) {
        if (c < '0' || c > '9') return false;
    }
    return true;
}

class CameraNode : public rclcpp::Node {
public:
    CameraNode() : Node("camera_node") {
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/camera/image_raw", 10);

        this->declare_parameter("source", "0");
        this->declare_parameter("fps", 30.0);
        this->declare_parameter("width", 640);
        this->declare_parameter("height", 480);

        update_vid();
        start_timer();
        param_cb_ = add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for(auto& param : params) {
                    if(param.get_name() == "source") update_vid();
                    else if(param.get_name() == "fps") start_timer();
                }
            }
        );
    }

private:
    void start_timer() {
        auto period = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(1.0 / this->get_parameter("fps").as_double()));
        if (timer_) timer_->cancel();
        timer_ = rclcpp::create_timer(this, get_clock(), period, [this]() { tick(); });
    }

    void update_vid() {
        vid_.release();
        if(isAllDigits(this->get_parameter("source").as_string())) {
            vid_.open(std::stoi(this->get_parameter("source").as_string()));
        } else {
            vid_.open(this->get_parameter("source").as_string());
        }

        if(!vid_.isOpened()) RCLCPP_ERROR(this->get_logger(), "Tidak bisa membuka kamera/file");
    }

    void tick() {
        if (!vid_.isOpened()) return;
        cv::Mat frame;
        vid_ >> frame;

        if(frame.empty()) {
            vid_.set(cv::CAP_PROP_POS_FRAMES, 0);
            vid_ >> frame;
        }

        if(!isAllDigits(this->get_parameter("source").as_string())) cv::flip(frame, frame, 0);
        cv::resize(frame, frame, cv::Size(this->get_parameter("width").as_int(), this->get_parameter("height").as_int()));
        auto msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", frame).toImageMsg();
        msg->header.stamp = this->now();
        publisher_->publish(*msg);
    }

    cv::VideoCapture vid_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CameraNode>());
    rclcpp::shutdown();
    return 0;
}