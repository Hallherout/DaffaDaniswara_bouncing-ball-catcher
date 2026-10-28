#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/header.hpp"
#include "opencv2/opencv.hpp"
#include "cv_bridge/cv_bridge.hpp"
#include <vector>
#include <random>
#include <cmath>

class GameNode : public rclcpp::Node {
public:
    GameNode() : Node("game_node"), score(0) {
        this->declare_parameter<std::vector<int>>("hsv_lower", {85, 70, 70});
        this->declare_parameter<std::vector<int>>("hsv_upper", {105, 255, 255});
        this->declare_parameter("num_balls", 3);
        this->declare_parameter("hold_duration", 2.0);
        this->declare_parameter("ball_radius", 40);
        this->declare_parameter("ball_speed", 150.0);
        this->declare_parameter("min_area", 300);

        pub_score = this->create_publisher<std_msgs::msg::Int32>("/game/score", 10);
        pub_img = this->create_publisher<sensor_msgs::msg::Image>("/game/debug_image", 10);

        subscribtion_ = this->create_subscription<sensor_msgs::msg::Image>("/camera/image_raw", 10, std::bind(&GameNode::image_callback, this, std::placeholders::_1));

        param_cb_ = add_post_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter>& params) {
                for(auto& param : params) {
                    if(param.get_name() == "ball_speed") {
                        double new_speed = param.as_double();

                        for (size_t i = 0; i < ball_vel.size(); i++) {
                            double current_speed = std::hypot(ball_vel[i].x, ball_vel[i].y);
                            if (current_speed > 0.0) {
                                ball_vel[i].x = (ball_vel[i].x / current_speed) * new_speed;
                                ball_vel[i].y = (ball_vel[i].y / current_speed) * new_speed;
                            }
                        }
                    }
                }
            }
        );
    }

private:
    double rand_range(double min, double max) {
        static std::random_device rd;
        static std::mt19937 gen(rd());
        std::uniform_real_distribution<double> dis(min, max);
        return dis(gen);
    }

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
        cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, "bgr8");
        cv::Mat debug_img = cv_ptr->image, hsv_img;
        cv::cvtColor(debug_img, hsv_img, cv::COLOR_BGR2HSV);

        while((int)ball_pos.size() < this->get_parameter("num_balls").as_int()) {
            ball_pos.push_back(cv::Point2d(rand_range(0.0, debug_img.cols), rand_range(0.0, debug_img.rows)));

            double ang = rand_range(0.0, 2.0 * CV_PI);
            double speed = this->get_parameter("ball_speed").as_double();
            ball_vel.push_back(cv::Point2d(speed * std::cos(ang), speed * std::sin(ang)));

            timer.push_back(0.0);
        }
        if((int)ball_pos.size() > this->get_parameter("num_balls").as_int()) ball_pos.resize(this->get_parameter("num_balls").as_int());

        cv::Mat mask;
        cv::Scalar lower_bound(
            this->get_parameter("hsv_lower").as_integer_array()[0],
            this->get_parameter("hsv_lower").as_integer_array()[1],
            this->get_parameter("hsv_lower").as_integer_array()[2]
        );
        cv::Scalar upper_bound(
            this->get_parameter("hsv_upper").as_integer_array()[0],
            this->get_parameter("hsv_upper").as_integer_array()[1],
            this->get_parameter("hsv_upper").as_integer_array()[2]
        );
        cv::inRange(hsv_img, lower_bound, upper_bound, mask);

        cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
        cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
        cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        double max_area = 0.0;
        int idx = -1;
        for(size_t i = 0; i < contours.size(); i++) {
            double area = cv::contourArea(contours[i]);
            if(area >= this->get_parameter("min_area").as_int() && area > max_area) {
                max_area = area;
                idx = i;
            }
        }

        rclcpp::Time now(msg->header.stamp);
        double time_gap = 0.0;
        if(prev_time.seconds()) time_gap = (now - prev_time).seconds();
        prev_time = now;

        int radius = this->get_parameter("ball_radius").as_int();
        for(size_t i = 0; i < ball_pos.size(); i++) {
            ball_pos[i] += ball_vel[i] * time_gap;

            if(ball_pos[i].x - radius <= 0) {
                ball_pos[i].x = radius;
                ball_vel[i].x *= -1.0;
            } else if(ball_pos[i].x + radius >= debug_img.cols) {
                ball_pos[i].x = debug_img.cols - radius;
                ball_vel[i].x *= -1.0;
            }

            if(ball_pos[i].y - radius <= 0) {
                ball_pos[i].y = radius;
                ball_vel[i].y *= -1.0;
            } else if(ball_pos[i].y + radius >= debug_img.rows) {
                ball_pos[i].y = debug_img.rows - radius;
                ball_vel[i].y *= -1.0;
            }
        }

        int target = -1;
        if(idx != -1) {
            cv::Rect bounding_box = cv::boundingRect(contours[idx]);
            cv::rectangle(debug_img, bounding_box, cv::Scalar(0, 255, 0), 2);

            cv::Moments m = cv::moments(contours[idx]);
            cv::Point2d centroid(m.m10 / m.m00, m.m01 / m.m00);
            cv::circle(debug_img, centroid, 5, cv::Scalar(0, 255, 0), -1);

            double min_dist = DBL_MAX;
            for(size_t i = 0; i < ball_pos.size(); i++) {
                double dist = std::hypot(centroid.x - ball_pos[i].x, centroid.y - ball_pos[i].y);
                if(dist <= radius && dist < min_dist) {
                    min_dist = dist;
                    target = (int)i;
                }
            }
        }

        for(size_t i = 0; i < ball_pos.size(); i++) {
            if((int)i == target) {
                timer[i] += time_gap;
                if(timer[i] >= this->get_parameter("hold_duration").as_double()) {
                    score++;
                    pub_score->publish(std_msgs::msg::Int32().set__data(score));

                    ball_pos[i] = cv::Point2d(rand_range(0.0, debug_img.cols), rand_range(0.0, debug_img.rows));
                    double ang = rand_range(0.0, 2.0 * CV_PI);
                    double speed = this->get_parameter("ball_speed").as_double();
                    ball_vel[i] = cv::Point2d(speed * std::cos(ang), speed * std::sin(ang));
                    timer[i] = 0.0;
                }
            } else {
                timer[i] = 0.0;
            }

            cv::circle(debug_img, ball_pos[i], radius, cv::Scalar(255, 0, 0), -1);
            if(timer[i] > 0.0) {
                double ratio = std::min(timer[i] / this->get_parameter("hold_duration").as_double(), 1.0);
                cv::ellipse(debug_img, ball_pos[i], cv::Size(radius + 6, radius + 6), 0, -90, -90 + 360 * ratio, cv::Scalar(0, 165, 255), 3);
            }
        }
        cv::putText(debug_img, "Score: " + std::to_string(score), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(0, 255, 255), 2);

        auto img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", debug_img).toImageMsg();
        pub_img->publish(*img);
    }

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscribtion_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr pub_score;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr pub_img;
    std::vector<cv::Point2d> ball_pos;
    std::vector<cv::Point2d> ball_vel;
    std::vector<double> timer;
    rclcpp::Time prev_time{0, 0, RCL_ROS_TIME};
    rclcpp::node_interfaces::PostSetParametersCallbackHandle::SharedPtr param_cb_;
    int score;
};

int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GameNode>());
    rclcpp::shutdown();
}