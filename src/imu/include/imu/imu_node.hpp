#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include <iostream>
#include <fstream>


class ImuNode : public rclcpp::Node
{
    public:
        ImuNode();
    private:
        //node variables
        rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr IMU_Subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr Ground_Truth_Subscriber;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr IMU_Position_Publisher;

        //variables for imu_callback
        double Gesamtgeschwindigkeit;
        double delta;
        double IMU_Drehwinkelgeschwindigkeit;
        double Theta;
        sensor_msgs::msg::Imu::SharedPtr Vorherige_IMU_Nachricht;
        double position_x;
        double position_y;  
        double IMU_Beschleunigung_x;
        double IMU_Beschleunigung_y;
        double Geschwindigkeit_x;
        double Geschwindigkeit_y;

        double IMU_Differenz;
        double IMU_Theta_Differenz;

        //variables for pose_callback
        double Ground_Truth_x;
        double Ground_Truth_y;
        double Ground_Truth_Theta;

        //callback functions
        void imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};