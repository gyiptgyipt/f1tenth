#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <math.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <cmath>
#include <limit.hpp>

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

    std::vector<geometry_msgs::msg::Point> points;

    double car_pos_x , car_pos_y , car_yaw;
    double point_pos_x, point_pos_y;

    double closest_distance = NULL;
    std::vector<geometry_msgs::msg::Point> closest_waypoint;

    void update_waypoint(nav_msgs::msg::Path::SharedPtr msg){

        
        for (const auto& pose : msg->poses) {
            geometry_msgs::msg::Point point;
            point.x = pose.pose.position.x;
            point.y = pose.pose.position.y;
            point.z = pose.pose.position.z;
            this->points.push_back(point);
        }

    }


    void get_next_point(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg){
        
        double dist = 0.6604;

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

    
    void timerCallback(){
        double search_angle = 1.5; //85 degree
        double car_length = 0.3302;
        double min_dist = car_length * 2.0;
        
        for (const auto &point : points){     //getting waypoints 
            double dx = point.x - this->car_pos_x; 
            double dy = point.y - this->car_pos_y;
            double angle = atan2(dy, dx);
            double relative_angle = get_relative_angle(car_yaw, angle);
            double distance = sqrt(pow(dx,2) + pow(dx,2));

            if (abs(relative_angle) < search_angle && distance < closest_distance && distance > min_dist){
                    this->closest_distance = distance;
                    this->closest_waypoint.push_back(point);
            }

        //Let's viz Point
        auto marker = visualization_msgs::msg::Marker();
        marker.header.frame_id = "map";
        marker.header.stamp = this->now();
        marker.ns = "";
        marker.id = 1;
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.pose.orientation.w = 1.0;

        // Points to visualize
        
        geometry_msgs::msg::Point p;
        p.x = this->closest_waypoint.pose.position.x;
        p.y = this->closest_waypoint.pose.position.y;
        p.z = 0.0;
        marker.points.push_back(p);
        

        // Set the scale of the points
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;

        // Set the color of the points
        marker.color.a = 1.0;  // Alpha
        marker.color.r = 0.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        publisher_->publish(marker);
            

        }
    }

    double angle_check(double angle){
        while (angle > M_PI){
            angle -= 2 * M_PI;
            }
        while (angle < -M_PI){
            angle += 2 * M_PI;
            }
        return angle;
    }

    double get_relative_angle(double car_yaw,double angle){
        double relative_angle = angle - car_yaw;
        relative_angle = angle_check(relative_angle);
        return relative_angle;
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
