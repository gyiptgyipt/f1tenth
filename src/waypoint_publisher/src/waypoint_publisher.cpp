#include <chrono>
#include <cstdio>
#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <nav_msgs/msg/path.hpp>

using namespace std::chrono_literals;

class WaypointPublisher : public rclcpp::Node {
  public:
    WaypointPublisher()
    : Node("waypoint_publisher")
    {
      raceline_publisher_ = this->create_publisher<nav_msgs::msg::Path>("raceline_waypoints", 10);
      centerline_publisher_ = this->create_publisher<nav_msgs::msg::Path>("centerline_waypoints", 10);
      blend_publisher_ = this->create_publisher<nav_msgs::msg::Path>("blend_waypoints", 10);
      timer_ = this->create_wall_timer(2s, std::bind(&WaypointPublisher::timer_callback, this));

      //// raceline
      //std::string raceline_file_name = "Spa_raceline.csv";
      //std::string raceline_file_loc = get_file_loc(raceline_file_name);
      //RCLCPP_INFO(this->get_logger(), "Raceline file location: %s", raceline_file_loc.c_str());

      //raceline_path = get_waypoints(raceline_file_loc, raceline_file_name);
      //RCLCPP_INFO(this->get_logger(), "Loaded %d raceline waypoints", (int)raceline_path.poses.size());

      // centerline
      std::string centerline_file_name = "campusmap_path.csv";
      std::string centerline_file_loc = get_file_loc(centerline_file_name);
      RCLCPP_INFO(this->get_logger(), "Centerline file location: %s", centerline_file_loc.c_str());

      centerline_path = get_waypoints(centerline_file_loc, centerline_file_name);
      RCLCPP_INFO(this->get_logger(), "Loaded %d centerline waypoints", (int)centerline_path.poses.size());

      //// blend
      //blend_path = blend_paths(raceline_path, centerline_path, 0.6);

      this->timer_callback();
    }

    nav_msgs::msg::Path raceline_path;
    nav_msgs::msg::Path centerline_path;
    nav_msgs::msg::Path blend_path;

  private:
    void timer_callback() {
      if(raceline_path.poses.size() > 0) {
        raceline_publisher_->publish(raceline_path);
        RCLCPP_INFO(this->get_logger(), "Publishing %d raceline waypoints", (int)raceline_path.poses.size());
      }

      if(centerline_path.poses.size() > 0) {
        centerline_publisher_->publish(centerline_path);
        RCLCPP_INFO(this->get_logger(), "Publishing %d centerline waypoints", (int)centerline_path.poses.size());
      }

      if(blend_path.poses.size() > 0) {
        blend_publisher_->publish(blend_path);
        RCLCPP_INFO(this->get_logger(), "Publishing %d blend waypoints", (int)blend_path.poses.size());
      }
    }
    
    std::string get_file_loc(std::string file_name) {
      return std::string((std::string)ament_index_cpp::get_package_share_directory("waypoint_publisher") + "/data/" + file_name);
    }

    std::vector<double> get_orientation(geometry_msgs::msg::PoseStamped pose1, geometry_msgs::msg::PoseStamped pose2) {
      // returns a quaternion representing the orientation from pose1 to pose2
      double yaw = atan2(pose2.pose.position.y - pose1.pose.position.y, pose2.pose.position.x - pose1.pose.position.x);
      return {0.0, 0.0, sin(yaw/2.0), cos(yaw/2.0)};
    }

    nav_msgs::msg::Path get_waypoints(std::string filepath, std::string filename) {
      nav_msgs::msg::Path new_path;
      new_path.header.frame_id = "map";
      new_path.header.stamp = this->now();
      FILE *fp;
      fp = fopen(filepath.c_str(), "r");
      if (fp == NULL) {
        RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", filepath.c_str());
        return new_path;
      }
      enum Parse_type {CENTERLINE, MINCURV, RACELINE, NON};
      Parse_type line_type = NON;

      if(filename.find("centerline") != std::string::npos || filename.find("path") != std::string::npos) {
        line_type = CENTERLINE;
        RCLCPP_INFO(this->get_logger(), "Detected centerline or path from filename: %s", filename.c_str());
      } else if(filename.find("mincurv") != std::string::npos) {
        line_type = MINCURV;
        RCLCPP_INFO(this->get_logger(), "Detected mincurv from filename: %s", filename.c_str());
      } else if(filename.find("raceline") != std::string::npos) {
        line_type = RACELINE;
        RCLCPP_INFO(this->get_logger(), "Detected raceline from filename: %s", filename.c_str());
      } else {
        RCLCPP_WARN(this->get_logger(), "Failed to detect line type from filename: %s", filename.c_str());
      }

      char buf[256];
      while (fgets(buf, sizeof(buf), fp) != NULL) {
        // skip lines that start with #
        if(buf[0] == '#')
          continue;

        double x, y, w_tr_right, w_tr_left, v;
        if(line_type == CENTERLINE) {
          sscanf(buf, "%lf, %lf, %lf, %lf", &x, &y, &w_tr_right, &w_tr_left);
          v = 0;
        } else if(line_type == MINCURV) {
          sscanf(buf, "%lf,%lf,%lf", &x, &y, &v);
        } else if(line_type == RACELINE) {
          sscanf(buf, "%*f;%lf;%lf;%*f;%*f;%lf", &x, &y, &v);
        } else if(line_type == NON) {
          sscanf(buf, "%*f %lf %lf", &x, &y);
          v = 0;
        }
        geometry_msgs::msg::PoseStamped pose;
        pose.header = new_path.header;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = x;
        pose.pose.position.y = y;
        pose.pose.position.z = v;

        // orientation
        if (new_path.poses.size() > 0) {
          std::vector<double> orientation = get_orientation(new_path.poses[new_path.poses.size()-1], pose);
          pose.pose.orientation.x = orientation[0];
          pose.pose.orientation.y = orientation[1];
          pose.pose.orientation.z = orientation[2];
          pose.pose.orientation.w = orientation[3];
        } else {
          pose.pose.orientation.x = 0.0;
          pose.pose.orientation.y = 0.0;
          pose.pose.orientation.z = 0.0;
          pose.pose.orientation.w = 1.0;
        }
        new_path.poses.push_back(pose);
      }
      return new_path;
    }

    nav_msgs::msg::Path blend_paths(nav_msgs::msg::Path path1, nav_msgs::msg::Path path2, double blend_factor = 0.5) {
      // path1 should be centerline
      // path2 should be raceline
      nav_msgs::msg::Path new_path;
      new_path.header.frame_id = "map";
      new_path.header.stamp = this->now();
      // find shortest path
      nav_msgs::msg::Path &short_path = (path1.poses.size() < path2.poses.size()) ? path1 : path2;
      nav_msgs::msg::Path &long_path = (path1.poses.size() < path2.poses.size()) ? path2 : path1;
      for(size_t i = 0; i < short_path.poses.size(); i++) {
        geometry_msgs::msg::PoseStamped& current_pose = short_path.poses[i];
        // find closest point on other path
        double min_dist = 1000000;
        geometry_msgs::msg::PoseStamped* closest_pose;
        for(size_t j = 0; j < long_path.poses.size(); j++) {
          double dist = sqrt(pow(current_pose.pose.position.x - long_path.poses[j].pose.position.x, 2) + pow(current_pose.pose.position.y - long_path.poses[j].pose.position.y, 2));
          if(dist < min_dist) {
            min_dist = dist;
            closest_pose = &long_path.poses[j];
          }
        }
        // blend points
        geometry_msgs::msg::PoseStamped pose;
        pose.header = new_path.header;
        pose.header.stamp = this->now();
        pose.header.frame_id = "map";
        pose.pose.position.x = short_path.poses[i].pose.position.x + (closest_pose->pose.position.x - short_path.poses[i].pose.position.x) * blend_factor;
        pose.pose.position.y = short_path.poses[i].pose.position.y + (closest_pose->pose.position.y - short_path.poses[i].pose.position.y) * blend_factor;
        // take largest z
        pose.pose.position.z = std::max(short_path.poses[i].pose.position.z, closest_pose->pose.position.z);
        new_path.poses.push_back(pose);
      }
      return new_path;
    }

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr raceline_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr centerline_publisher_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr blend_publisher_;
};




int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<WaypointPublisher>());
  rclcpp::shutdown();
  return 0;
}
