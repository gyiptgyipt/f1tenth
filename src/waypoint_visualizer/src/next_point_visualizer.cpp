#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>

#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

using namespace std::chrono_literals;
using std::placeholders::_1;

class PointSpherePublisher : public rclcpp::Node
{
public:
    PointSpherePublisher()
    : Node("point_sphere_publisher")
    {
        publisher_ = this->create_publisher<visualization_msgs::msg::Marker>("/visualization_marker", 10);
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&PointSpherePublisher::get_next_point, this, _1));
        waypoint_sub_ = this->create_subscription<visualization_msgs::msg::Marker>("/centerline_waypoints",10,std::bind(&PointSpherePublisher::update_waypoint,this,_1));

        timer_ = this->create_wall_timer(500ms, std::bind(&PointSpherePublisher::timerCallback, this));
        
    }

private:

    const nav_msgs::msg::Path path_point;

    double point_pos_x, point_pos_y;

    void update_waypoint(nav_msgs::msg::Path msg){

        // path_point.pose.position.X = 0;

    }


    void get_next_point(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg){
        
        double dist = 0.6604;

        auto car_pos_x = odom_msg->pose.pose.position.x;
        auto car_pos_y = odom_msg->pose.pose.position.y;

        tf2::Quaternion quat(
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z,
            odom_msg->pose.pose.orientation.w);
    
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        
        
        
    }

    
    
    void timerCallback(){
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "my_namespace";
        marker.id = 0;
        marker.type = visualization_msgs::msg::Marker::SPHERE;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.position.x = 0.0;
        marker.pose.position.y = 0.0;
        marker.pose.position.z = 0.0;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;
        marker.scale.x = 1.0;
        marker.scale.y = 1.0;
        marker.scale.z = 1.0;
        marker.color.a = 1.0; // Don't forget to set the alpha!
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        // marker.color.h = 0.0;

        publisher_->publish(marker);
    }


    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<visualization_msgs::msg::Marker>::SharedPtr waypoint_sub_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PointSpherePublisher>());
    rclcpp::shutdown();
    return 0;
}
