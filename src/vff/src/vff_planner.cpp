#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <vector>
#include <limits>
#include <cmath>

#inlcude "vff/pid.cpp"

class VFFPlanner : public rclcpp::Node {
public:
    VFFPlanner() : Node("vff_node") {
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&VFFPlanner::scanCallback, this, std::placeholders::_1));
        drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    }

private:
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        std::vector<float> histogram(360, 0.0);

        // Populate the histogram based on laser scan data
        for (size_t i = 0; i < msg->ranges.size(); ++i) {
            float range = msg->ranges[i];
            if (range < msg->range_max && range > msg->range_min) {
                int angle = static_cast<int>((msg->angle_min + i * msg->angle_increment) * 180.0 / M_PI);
                if (angle < 0) angle += 360;
                histogram[angle] += 1.0 / range;  // Inverse of range to prioritize closer obstacles
            }
        }

        // Find candidate directions
        int best_direction = -1;
        float best_value = std::numeric_limits<float>::max();
        for (int i = 0; i < histogram.size(); ++i) {
            if (histogram[i] < best_value) {
                best_value = histogram[i];
                best_direction = i;
            }
        }

        // Convert the best direction to a steering command
        auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
        if (best_direction != -1) {
            float steering_angle = (best_direction - 180) * M_PI / 180.0;
            drive_msg.drive.speed = 1.0;  // Set a constant speed
            drive_msg.drive.steering_angle = steering_angle * 0.5;  // Scale the steering angle
        } else {
            drive_msg.drive.speed = 0.0;
            drive_msg.drive.steering_angle = 0.0;
        }

        // Publish the drive command
        drive_pub_->publish(drive_msg);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VFFPlanner>());
    rclcpp::shutdown();
    return 0;
}
