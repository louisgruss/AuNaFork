#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//includes for tf matrix
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

#include <iostream>
#include <fstream>


class ImuNode : public rclcpp::Node
{
    public:
        ImuNode();
    private:
        //node variables
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pred_pose_imu;

        //variables for imu_callback
        double imu_acceleration_magnitude;
        double velocity;
        double delta;
        double imu_yaw_rate_;
        double theta;
        builtin_interfaces::msg::Time t;
        sensor_msgs::msg::Imu::SharedPtr last_imu_msg_;
        double pose_x = 0.0;
        double pose_y = 0.0;  
        double imu_acceleration_x;
        double imu_acceleration_y;
        double velocity_x;
        double velocity_y;
        double last_imu_acceleration_magnitude;
        double last_imu_velocity;

        //variables for pose_callback
        double pose_x_;
        double pose_y_;
        double pose_yaw_;

        tf2::Quaternion q_;
        tf2::Matrix3x3 m_;
        double roll_;
        double pitch_;
        double yaw_;
        double pose_yaw;

        //callback functions
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};