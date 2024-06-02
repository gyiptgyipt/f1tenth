#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <algorithm>
#include <vector>
#include <limits>
#include <cmath>

class FollowTheGap : public rclcpp::Node {
public:
    FollowTheGap()
    : Node("follow_the_gap") {
        lidar_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&FollowTheGap::lidar_callback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

private:
    void lidar_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float> ranges = msg->ranges;

        preprocess_lidar(ranges);

        int closest = std::distance(ranges.begin(), std::min_element(ranges.begin(), ranges.end()));
        create_bubble(ranges, closest, bubble_radius_);

        auto [start_i, end_i] = find_max_gap(ranges);

        int best_point = (start_i + end_i) / 2;

        float steering_angle = msg->angle_min + best_point * msg->angle_increment;

        geometry_msgs::msg::Twist drive_msg;
        drive_msg.linear.x = 1.0;  // Constant speed
        drive_msg.angular.z = steering_angle;
        drive_pub_->publish(drive_msg);
    }

    void preprocess_lidar(std::vector<float> &ranges) {
        float max_range_ = 3.0 ;
        std::replace_if(ranges.begin(), ranges.end(), [](float range) { return std::isnan(range) || range > max_range_; }, 0.0f);
        std::vector<float> smoothed_ranges(ranges.size());
        std::fill(smoothed_ranges.begin(), smoothed_ranges.end(), 0.0f);

        for (size_t i = 0; i < ranges.size(); ++i) {
            if (ranges[i] == 0.0f) continue;
            int count = 0;
            for (int j = -preprocess_conv_size_ / 2; j <= preprocess_conv_size_ / 2; ++j) {
                if (i + j >= 0 && i + j < ranges.size()) {
                    smoothed_ranges[i] += ranges[i + j];
                    ++count;
                }
            }
            smoothed_ranges[i] /= count;
        }

        ranges = smoothed_ranges;
    }

    void create_bubble(std::vector<float> &ranges, int closest, int bubble_radius) {
        for (int i = -bubble_radius; i <= bubble_radius; ++i) {
            if (closest + i >= 0 && closest + i < ranges.size()) {
                ranges[closest + i] = 0.0f;
            }
        }
    }

    std::pair<int, int> find_max_gap(const std::vector<float> &ranges) {
        int max_start = 0, max_end = 0;
        int current_start = 0;

        while (current_start < ranges.size()) {
            while (current_start < ranges.size() && ranges[current_start] == 0.0f) {
                ++current_start;
            }
            int current_end = current_start;
            while (current_end < ranges.size() && ranges[current_end] != 0.0f) {
                ++current_end;
            }
            if (current_end - current_start > max_end - max_start) {
                max_start = current_start;
                max_end = current_end;
            }
            current_start = current_end;
        }

        return {max_start, max_end};
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr drive_pub_;

    const int bubble_radius_ = 20;
    const int preprocess_conv_size_ = 3;
    const float max_range_ = 3.0f;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FollowTheGap>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
