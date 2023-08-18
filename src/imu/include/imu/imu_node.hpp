#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/Imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//includes for tf matrix
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

class imu_node : public rclcpp::Node
{
    public:
        imu_node();
    private:
        //node variables
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;

        //variables for imu_callback
        double imu_acceleration;

        //variables for pose_callback
        double pose_x_;
        double pose_y_;
        double pose_yaw_;

        tf2::Quaternion q_;
        tf2::Matrix3x3 m_;
        double roll_;
        double pitch_;
        double yaw_;

        //callback functions
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};