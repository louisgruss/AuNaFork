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
#include <iostream>
#include <fstream>


namespace ekf{
class EKFNode : public rclcpp::Node
{
    public:
        EKFNode();
    private:
        //node variables
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr Ground_Truth_Subscriber;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr EKF_Publisher;
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odometrie_Subscriber;
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMU_Subscriber;

        bool is_init;

        double Odom_Geschwindigkeit_x;
        double Odom_Geschwindigkeit_y;
        double Odom_Geschwindigkeit;
        double Odometrie_Drehwinkelgeschwindigkeit;

        double IMU_Beschleunigung_x;
        double IMU_Beschleunigung_y;
        double IMU_Beschleunigung;
        double IMU_Drehwinkelgeschwindigkeit;

        double dt;

        builtin_interfaces::msg::Time t;

        Eigen::VectorXd x;
        Eigen::VectorXd z;
        Eigen::MatrixXd P;
        Eigen::MatrixXd Q;
        Eigen::MatrixXd F;
        Eigen::MatrixXd R;
        Eigen::MatrixXd H1;
        Eigen::MatrixXd H2;
        Eigen::VectorXd y;
        Eigen::MatrixXd S;
        Eigen::MatrixXd K;
        Eigen::MatrixXd I;
        Eigen::MatrixXd J;

        bool flag;

        void Init();
        void Praediktion(double dt);
        void Korrektur(Eigen::MatrixXd Hj);

        double EKF_Differenz;
        double EKF_Theta_Differenz;

        double Ground_Truth_x;
        double Ground_Truth_y;
        double Ground_Truth_Theta;

        //callback functions       
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);  
};
}