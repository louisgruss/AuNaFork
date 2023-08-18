#include "imu_node.hpp"

imu_node::imu_node() : Node("imu_node")
{
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu_msg", 2, [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->imu_callback(msg);});
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
}

void imu_node::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_acceleration = imu_msg.imu.linear_acceleration.x;
}

void imu_node::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    pose_x_ = msg->pose.position.x;
    pose_y_ = msg->pose.position.y;

    q_.setW(msg->pose.orientation.w);
    q_.setX(msg->pose.orientation.x);
    q_.setY(msg->pose.orientation.y);
    q_.setZ(msg->pose.orientation.z);
    q_.normalize();
    m_.setRotation(q_);
    m_.getRPY(roll_, pitch_, yaw_);
    pose_yaw_ = yaw_;
}

