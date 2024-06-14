#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;

class PointSpherePublisher : public rclcpp::Node
{
public:
    PointSpherePublisher()
    : Node("point_sphere_publisher")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PointSpherePublisher::get_next_point, this,std::placeholders::_1));
        waypoint_sub_ = this->create_subscription<nav_msgs::msg::Path>("/centerline_waypoints", 10, std::bind(&PointSpherePublisher::update_waypoint, this,std::placeholders::_1));

        timer_ = this->create_wall_timer(500ms, std::bind(&PointSpherePublisher::timerCallback, this));
    }

private:
    std::vector<geometry_msgs::msg::Point> points;
    double car_pos_x, car_pos_y, car_yaw;
    double closest_distance = std::numeric_limits<double>::max();
    std::vector<geometry_msgs::msg::Point> closest_waypoint;

    void update_waypoint(const nav_msgs::msg::Path::SharedPtr msg)
    {
        for (const auto &pose : msg->poses) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            this->points.push_back(point);
        }
    }

    void get_next_point(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
    {
        this->car_pos_x = odom_msg->pose.pose.position.x;
        this->car_pos_y = odom_msg->pose.pose.position.y;

        tf2::Quaternion quat(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);

        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        this->car_yaw = yaw;
    }

    void timerCallback()
    {
        double search_angle = 1.5; // 85 degrees
        double car_length = 0.3302;
        double min_dist = car_length * 2.0;
        double closest_distance = std::numeric_limits<double>::max();
        geometry_msgs::msg::Point closest_point;

        for (const auto &point : points) { // getting waypoints
            double dx = point.x - this->car_pos_x;
            double dy = point.y - this->car_pos_y;
            double angle = atan2(dy, dx);
            double relative_angle = get_relative_angle(car_yaw, angle);
            double distance = sqrt(pow(dx, 2) + pow(dy, 2)); // Use dx and dy

            if (abs(relative_angle) < search_angle && distance < closest_distance && distance > min_dist) {
                closest_distance = distance;
                closest_point = point;
            }
        }

        // Let's visualize the closest Point
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "waypoints";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position = closest_point;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        publisher_->publish(marker);
    }

    double angle_check(double angle)
    {
        while (angle > M_PI) {
            angle -= 2 * M_PI;
        }
        while (angle < -M_PI) {
            angle += 2 * M_PI;
        }
        return angle;
    }

    double get_relative_angle(double car_yaw, double angle)
    {
        double relative_angle = angle - car_yaw;
        relative_angle = angle_check(relative_angle);
        return relative_angle;
    }

    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr waypoint_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointSpherePublisher>());
    rclcpp::shutdown();
    return 0;
}
