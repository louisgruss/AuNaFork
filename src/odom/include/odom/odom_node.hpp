#include "rclcpp/rclcpp.hpp"

#include "std_msgs/msg/float64.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "sensor_msgs/msg/imu.hpp"

#include <iostream>
#include <fstream>



class OdomNode : public rclcpp::Node
{
    public:
        OdomNode();
    private:
        //node variables
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr Odometrie_Subscriber;
        rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr Ground_Truth_Subscriber;
        rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr Odometrie_Position_Publisher;

        //variables for odom_callback
        double Odometrie_Geschwindigkeit;
        double Odometrie_Drehwinkelgeschwindigkeit;
        double delta;
        double position_x;
        double position_y;
        double Theta;

        double Odom_Differenz;
        double Odometrie_Theta_Differenz;


        nav_msgs::msg::Odometry::SharedPtr Vorherige_Odometrienachricht;

        //variables for pose_callback
        double Ground_Truth_x;
        double Ground_Truth_y ;
        double Ground_Truth_Theta;

        //callback functions
        void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
        void pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
};
