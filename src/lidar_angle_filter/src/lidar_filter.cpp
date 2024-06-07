#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

class LidarFilterNode : public rclcpp::Node
{
public:
    LidarFilterNode() : Node("lidar_filter_node")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "/scan", 10, std::bind(&LidarFilterNode::listener_callback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>("/filtered_scan", 10);
    }

private:
    void listener_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg)
    {
        auto filtered_scan = std::make_shared<sensor_msgs::msg::LaserScan>(*msg);
        filtered_scan->ranges.clear();

        double angle_min = -60.0 * M_PI / 180.0;  // -60 degrees in radians
        double angle_max = 60.0 * M_PI / 180.0;   // 60 degrees in radians
        double current_angle = msg->angle_min;

        for (const auto &range : msg->ranges)
        {
            if (current_angle >= angle_min && current_angle <= angle_max)
            {
                filtered_scan->ranges.push_back(range);
            }
            else
            {
                filtered_scan->ranges.push_back(std::numeric_limits<float>::infinity());
            }
            current_angle += msg->angle_increment;
        }

        publisher_->publish(*filtered_scan);
    }

    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr publisher_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LidarFilterNode>());
    rclcpp::shutdown();
    return 0;
}
