#include "ekf/ekf_node.hpp"

namespace ekf{

EKFNode::EKFNode() : Node("ekf_node")
{
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    //sub_pred_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pred_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pred_pose_callback(msg);});
    //sub_pred_pose_imu_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pred_pose_imu", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pred_pose_imu_callback(msg);});
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10).best_effort(), [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->imu_callback(msg);});
    pub_ekf_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() -> void { this->timer_callback(); });
}

void EKFNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
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

void EKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_acceleration_x = msg->linear_acceleration.x;
    imu_acceleration_y = msg->linear_acceleration.y;
    imu_yaw_rate = msg->angular_velocity.z;

    Predict(0.005);
    imu_c ++;

    /*z_ << this->odom_velocity_x, this->imu_acceleration_x, this->odom_velocity_y, this->imu_acceleration_y, (this->imu_yaw_rate + this->odom_yaw_rate) / 2;

    Predict(0.005);
    Update();*/

    /*this->y_ = z_imu - this->H_ * this->x_;
    this->S_ = this->H_* this->P_ * this->H_.transpose() + this->R_;
    this->K_ = this->P_ * this->H_.transpose() * this->S_.inverse();
    this->x_ = this->x_ + this->K_* this->y_;
    this->P_ = (this->I_ - this->K_ * this->H_) * this->P_;*/
}

void EKFNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    t = msg->header.stamp;    
    odom_velocity_x = msg->twist.twist.linear.x;
    odom_velocity_y = msg->twist.twist.linear.y;
    odom_yaw_rate = msg->twist.twist.angular.z;

    //Predict(0.01);
    odom_c ++;

    /*z_ << this->odom_velocity_x, this->imu_acceleration_x, this->odom_velocity_y, this->imu_acceleration_y, (this->imu_yaw_rate + this->odom_yaw_rate) / 2;

    Predict(0.01);
    Update();*/

    /*this->y_ = z_odom - this->H_ * this->x_;
    this->S_ = this->H_* this->P_ * this->H_.transpose() + this->R_;
    this->K_ = this->P_ * this->H_.transpose() * this->S_.inverse();
    this->x_ = this->x_ + this->K_* this->y_;
    this->P_ = (this->I_ - this->K_ * this->H_) * this->P_;*/
}

void EKFNode::timer_callback()
{
    if(!is_init)
    {
        Init();
    }
    else
    {
        /*
        //Predict
        //Predict(this->x_, this->P_, this->Q_, this->F_);
        //x_ = F_ * x_;
        //P_ = F_ * P_ * F_.transpose() + Q_;

        x_ = F_ * x_;
        P_ = F_ * P_ * F_.transpose() + Q_;
        //Update
        //Update(this->x_, this->P_, this->S_, this->H_, this->R_, this->z_odom);
        z_ << this->odom_velocity_x, this->imu_acceleration_x, this->odom_velocity_y, this->imu_acceleration_y, (this->imu_yaw_rate + this->odom_yaw_rate) / 2;


        y_ = z_ - H_ * x_;
        S_ = H_* P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S_.inverse();

        x_ = x_ + K_* y_;
        //P_ = P_ - K_ * H_ * P_;
        P_ = (I_ - K_ * H_) * P_;

        //Update(this->x_, this->P_, this->S_, this->H_, this->R_, this->z_imu);

        y_ = z_ - H_ * x_;
        S_ = H_* P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S_.inverse();

        x_ = x_ + K_* y_;
        P_ = P_ - K_ * H_ * P_;


        
        z_ << this->odom_velocity_x, this->imu_acceleration_x, this->odom_velocity_y, this->imu_acceleration_y, (this->imu_yaw_rate + this->odom_yaw_rate) / 2;

        F_ <<
        1.0, dt, 0.5 * pow(dt, 2), 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
        0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

        0.0, 0.0, 0.0, 1.0, dt, 0.5 * pow(dt, 2), 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0,
        0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt,
        0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

        x_ = F_ * x_;

        J_ <<
        1, cos(x_(6)) * dt, 0, 0, 0, 0, -(x_(1)) * sin(x_(6)) * dt, 0, 
        0, 1, dt, 0, 0, 0, 0, 0,
        0, 0, 1, 0, 0, 0, 0, 0, 

        0, sin(x_(6)) * dt, 0, 1, 0, 0, x_(1) * cos(x_(6)) * dt, 0, 
        0, 0, 0, 0, 1, dt, 0, 0,
        0, 0, 0, 0, 0, 1, 0, 0,

        0, 0, 0, 0, 0, 0, 1, dt,
        0, 0, 0, 0, 0, 0, 0, 1;
        
        P_ = J_ * P_ * J_.transpose() + Q_;

        y_ = z_ - H_ * x_;
        S_ = H_* P_ * H_.transpose() + R_;
        K_ = P_ * H_.transpose() * S_.inverse();

        x_ = x_ + K_* y_;
        P_ = (I_ - K_ * H_) * P_;*/

        z_ << this->odom_velocity_x, this->imu_acceleration_x, this->odom_velocity_y, this->imu_acceleration_y, (this->imu_yaw_rate + this->odom_yaw_rate) / 2;

        Update();

        geometry_msgs::msg::PoseStamped ekf;    
        ekf.header.frame_id = "ekf_frame_";
        ekf.header.stamp = this->t;

        ekf.pose.position.x = x_(0);
        ekf.pose.position.y = x_(3);
        ekf.pose.position.z = 0.0;

        ekf.pose.orientation.x = 0.0;
        ekf.pose.orientation.y = 0.0;
        ekf.pose.orientation.z = x_(6)/2;
        ekf.pose.orientation.w = 1.0;

        this->pub_ekf_->publish(ekf);
    }
}

/*autodiff::VectorXreal EKFNode::propagate(Eigen::VectorXd x_)
{
    autodiff::VectorXreal newx_(x_.size());
    newx_(0) = x_(0) + x_(1) * cos(x_(6)) * delta;
    newx_(1) = x_(1) + x_(2) * delta;    
    newx_(2) = x_(2);  
    newx_(3) = x_(3) + x_(1) * sin(x_(6)) * delta;
    newx_(4) = x_(4) + x_(5) * delta;    
    newx_(5) = x_(5);
    newx_(6) = x_(6) + x_(7) * delta;
    newx_(7) = x_(7);
    return newx_;
}*/

void EKFNode::Init()
{
    x_ = Eigen::VectorXd(8);
    P_ = Eigen::MatrixXd(8,8);
    Q_ = Eigen::MatrixXd(8,8);
    F_ = Eigen::MatrixXd(8,8);
    H_ = Eigen::MatrixXd(5,8);
    R_ = Eigen::MatrixXd(5,5);
    y_ = Eigen::VectorXd(5);
    K_ = Eigen::MatrixXd(8,5);
    z_ = Eigen::VectorXd(5);
    J_ = Eigen::MatrixXd(8,8);
    I_ = Eigen::MatrixXd::Identity(8,8);

    dt = 0.01;

    //Initialization
    //Init(this->pose_x, this->pose_y, this->pose_yaw, this->delta);
    x_ << this->pose_x, this->odom_velocity_x, this->imu_acceleration_x, 
    this->pose_y, this->odom_velocity_y, this->imu_acceleration_y, 
    this->pose_yaw, (this->imu_yaw_rate + this->odom_yaw_rate) / 2;

    P_ << 
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 100.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 100.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 100.0;

    Q_ << 
    0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.02;

    H_ << 
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0; 

    R_ << 
    0.000001, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.000001, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.000001, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.000001, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.000001;

    is_init = true;
}

void EKFNode::Predict(double dt)
{
    F_ <<
    1.0, dt, 0.5 * pow(dt, 2), 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 1.0, dt, 0.5 * pow(dt, 2), 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    x_ = F_ * x_;

    J_ <<
    1, cos(x_(6)) * dt, 0, 0, 0, 0, -(x_(1)) * sin(x_(6)) * dt, 0, 
    0, 1, dt, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0, 

    0, sin(x_(6)) * dt, 0, 1, 0, 0, x_(1) * cos(x_(6)) * dt, 0, 
    0, 0, 0, 0, 1, dt, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,

    0, 0, 0, 0, 0, 0, 1, dt,
    0, 0, 0, 0, 0, 0, 0, 1;
    
    P_ = J_ * P_ * J_.transpose() + Q_;
}

void EKFNode::Update()
{
    y_ = z_ - H_ * x_;
    S_ = H_* P_ * H_.transpose() + R_;
    K_ = P_ * H_.transpose() * S_.inverse();

    x_ = x_ + K_* y_;
    P_ = (I_ - K_ * H_) * P_;
}
}