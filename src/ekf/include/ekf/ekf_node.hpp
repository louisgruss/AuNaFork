#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "Eigen/Dense"
#include "Eigen/Core"
#include "Eigen/Geometry"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"
#include <autodiff/forward/real.hpp>
#include <autodiff/forward/real/eigen.hpp>

namespace ekf{
class EKFNode : public rclcpp::Node
{
    public:
        EKFNode();
    private:
        //node variables
        //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pred_pose_;
        //rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pred_pose_imu_;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_pose_stamped_;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_ekf_;
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;

        double pose_x;
        double pose_y;
        tf2::Quaternion q_;
        tf2::Matrix3x3 m_;
        double roll_;
        double pitch_;
        double yaw_;
        double pose_yaw;

        bool is_init;

        double odom_x;
        double odom_y;
        double odom_theta;
        double odom_velocity_x;
        double odom_velocity_y;
        double odom_yaw_rate;

        double imu_x;
        double imu_y;
        double imu_yaw_rate;
        double imu_acceleration_x;
        double imu_acceleration_y;

        double delta;
        double dt;
        double imu_c;
        double odom_c;

        Eigen::VectorXd z_odom; 
        Eigen::VectorXd z_imu;
        Eigen::VectorXd z_;

        builtin_interfaces::msg::Time t;

        Eigen::VectorXd x_;
        Eigen::MatrixXd P_;
        Eigen::MatrixXd Q_;
        Eigen::MatrixXd F_;
        Eigen::MatrixXd R_;
        Eigen::MatrixXd H_;
        Eigen::VectorXd y_;
        Eigen::MatrixXd S_;
        Eigen::MatrixXd K_;
        Eigen::MatrixXd I_;
        Eigen::MatrixXd J_;

        void Init();
        void Predict(double dt);
        void Update();

        //callback functions
        //void pred_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        //void pred_pose_imu_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);        
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void timer_callback();
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
};
}