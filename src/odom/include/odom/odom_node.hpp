#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

//includes for tf matrix
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <fstream>



class OdomNode : public rclcpp::Node
{
    public:
        OdomNode();
    private:
        //node variables
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pred_pose;

        //variables for odom_callback
        double odom_velocity_;
        double odom_yaw_rate_;
        double delta;
        builtin_interfaces::msg::Time t;

        nav_msgs::msg::Odometry::SharedPtr last_odom_msg_;

        //variables for pose_callback
        double pose_x_;
        double pose_y_ ;
        double pose_yaw_;

        tf2::Quaternion q_;
        tf2::Matrix3x3 m_;
        double roll_;
        double pitch_;
        double yaw_;
        double pose_yaw;
        double pose_x;
        double pose_y;

        //callback functions
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};
