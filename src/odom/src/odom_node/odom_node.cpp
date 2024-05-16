#include "odom/odom_node.hpp"


OdomNode::OdomNode() : Node("odom_node")
{
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    pub_pred_pose = this->create_publisher<geometry_msgs::msg::PoseStamped>("pred_pose", 10);
}

void OdomNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    odom_velocity_ = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)) * ((msg->twist.twist.linear.x < 0) ? -1 : 1);
    odom_yaw_rate_ = msg->twist.twist.angular.z;
    t = msg->header.stamp;

    if(last_odom_msg_ == nullptr)
    {
        last_odom_msg_ = msg;
        pose_x_ = this->pose_x;
        pose_y_ = this->pose_y;
        pose_yaw_ = this->pose_yaw;
        delta = 0;
    }
    else
    {
        //delta = (msg->header.stamp.sec - last_odom_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_odom_msg_->header.stamp.nanosec) / 1000000000.0);
        auto dt = rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_odom_msg_->header.stamp);
        delta = dt.seconds();
        last_odom_msg_ = msg;
        pose_x_ += odom_velocity_ * cos(pose_yaw_) * delta;
        pose_y_ += odom_velocity_ * sin(pose_yaw_) * delta;
        pose_yaw_ += odom_yaw_rate_ * delta;
    }
    geometry_msgs::msg::PoseStamped pred_pose;
    pred_pose.header.frame_id = "pred_pose_frame_";
    pred_pose.header.stamp = this->t;
    //pred_pose.child_frame_id = base_frame_;

    pred_pose.pose.position.x = this->pose_x_;
    pred_pose.pose.position.y = this->pose_y_;
    pred_pose.pose.position.z = 0.0;

    pred_pose.pose.orientation.x = 0.0;
    pred_pose.pose.orientation.y = 0.0;
    pred_pose.pose.orientation.z = sin(this->pose_yaw_/2.0);
    pred_pose.pose.orientation.w = 1.0;

    this->pub_pred_pose->publish(pred_pose);
}

void OdomNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    pose_x = msg->pose.position.x;
    pose_y = msg->pose.position.y;

    q_.setW(msg->pose.orientation.w);
    q_.setX(msg->pose.orientation.x);
    q_.setY(msg->pose.orientation.y);
    q_.setZ(msg->pose.orientation.z);
    q_.normalize();
    m_.setRotation(q_);
    m_.getRPY(roll_, pitch_, yaw_);
    pose_yaw = yaw_;
}
