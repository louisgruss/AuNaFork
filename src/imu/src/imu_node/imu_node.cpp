#include "imu/imu_node.hpp"

ImuNode::ImuNode() : Node("imu_node")
{
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10).best_effort(), [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->imu_callback(msg);});
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    pub_pred_pose_imu = this->create_publisher<geometry_msgs::msg::PoseStamped>("pred_pose_imu", 10);
    //timer_ = this->create_wall_timer(std::chrono::milliseconds(100), [this]() -> void { this->timer_callback(); });
}

void ImuNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_acceleration_magnitude = sqrt(pow(msg->linear_acceleration.x, 2) + pow(msg->linear_acceleration.y, 2)) * ((msg->linear_acceleration.x < 0) ? -1 : 1); 
    imu_acceleration_x = msg->linear_acceleration.x;
    imu_acceleration_y = msg->linear_acceleration.y;
    imu_yaw_rate_ = msg->angular_velocity.z;
    t = msg->header.stamp;
    theta = msg->orientation.z;     

    if(last_imu_msg_ == nullptr)
    {
        pose_x = this->pose_x_;
        pose_y = this->pose_y_;
        velocity_x = 0.0;
        velocity_y = 0.0;
        velocity = 0.0;
        last_imu_msg_ = msg;
        last_imu_acceleration_magnitude = 0.0;
        last_imu_velocity = 0.0;
    }
    else
    {
        //delta = (msg->header.stamp.sec - last_imu_msg_->header.stamp.sec + (msg->header.stamp.nanosec - last_imu_msg_->header.stamp.nanosec) / 1000000000.0); 
        auto dt = rclcpp::Time(msg->header.stamp) - rclcpp::Time(last_imu_msg_->header.stamp);
        delta = dt.seconds();
        pose_yaw += imu_yaw_rate_ * delta;
        //velocity += imu_acceleration_magnitude * delta;
        velocity += (imu_acceleration_magnitude + last_imu_acceleration_magnitude) * delta / 2;
        velocity_x += imu_acceleration_x * delta;
        velocity_y += imu_acceleration_y * delta;
        //pose_x += velocity_x * delta;
        //pose_y += velocity_y * delta;
        pose_x += (velocity * cos(theta) + last_imu_velocity * cos(theta)) * delta / 2.0;
        pose_y += (velocity * sin(theta) + last_imu_velocity * sin(theta)) * delta / 2.0;
        last_imu_msg_ = msg;      
        last_imu_acceleration_magnitude = imu_acceleration_magnitude;
        last_imu_velocity = velocity;
    }
    geometry_msgs::msg::PoseStamped pred_pose_imu;    
    pred_pose_imu.header.frame_id = "pred_pose_imu_frame_";
    pred_pose_imu.header.stamp = this->t;
    //pred_pose.child_frame_id = base_frame_;

    pred_pose_imu.pose.position.x = pose_x; //this->velocity * cos(pose_yaw) * this->delta + pose_x_;
    pred_pose_imu.pose.position.y = pose_y; //this->velocity * sin(pose_yaw) * this->delta + pose_y_;
    pred_pose_imu.pose.position.z = 0.0;

    pred_pose_imu.pose.orientation.x = 0.0;
    pred_pose_imu.pose.orientation.y = 0.0;
    pred_pose_imu.pose.orientation.z = this->theta;
    pred_pose_imu.pose.orientation.w = 1.0;

    this->pub_pred_pose_imu->publish(pred_pose_imu);
}

/*void ImuNode::timer_callback()
{
    geometry_msgs::msg::PoseStamped pred_pose_imu;    
    pred_pose_imu.header.frame_id = "pred_pose_imu_frame_";
    pred_pose_imu.header.stamp = this->t;
    //pred_pose.child_frame_id = base_frame_;

    pred_pose_imu.pose.position.x = this->pose_x; //this->velocity * cos(pose_yaw) * this->delta + pose_x_;
    pred_pose_imu.pose.position.y = this->pose_y; //this->velocity * sin(pose_yaw) * this->delta + pose_y_;
    pred_pose_imu.pose.position.z = this->delta;

    pred_pose_imu.pose.orientation.x = 0.0;
    pred_pose_imu.pose.orientation.y = 0.0;
    pred_pose_imu.pose.orientation.z = this->theta;
    pred_pose_imu.pose.orientation.w = 1.0;

    this->pub_pred_pose_imu->publish(pred_pose_imu);
}*/

void ImuNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

