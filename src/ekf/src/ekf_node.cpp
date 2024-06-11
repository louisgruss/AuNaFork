#include "ekf/ekf_node.hpp"

namespace ekf{

EKFNode::EKFNode() : Node("ekf_node")
{
    sub_pose_stamped_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    sub_imu_ = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10).best_effort(), [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->imu_callback(msg);});
    pub_ekf_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf", 10);
    timer_ = this->create_wall_timer(std::chrono::milliseconds(10), [this]() -> void { this->timer_callback(); });
    pub_goal_pose_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("goal_pose", 10);
    sub_pred_pose_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pred_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pred_pose_callback(msg);});
    sub_pred_pose_imu_ = this->create_subscription<geometry_msgs::msg::PoseStamped>("pred_pose_imu", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pred_pose_imu_callback(msg);});
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

void EKFNode::pred_pose_imu_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    imu_pose_x = msg->pose.position.x;
    imu_pose_y = msg->pose.position.y;
    imu_theta = msg->pose.orientation.z;
}

void EKFNode::pred_pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    odom_pose_x = msg->pose.position.x;
    odom_pose_y = msg->pose.position.y;
    odom_theta = msg->pose.orientation.z;

}

void EKFNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    t = msg->header.stamp;    
    odom_velocity_x = msg->twist.twist.linear.x;
    odom_velocity_y = msg->twist.twist.linear.y;
    odom_velocity = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2)) * ((msg->twist.twist.linear.x < 0) ? -1 : 1);
    odom_yaw_rate = msg->twist.twist.angular.z;

    flag = true;

    //z_ << this->odom_velocity * cos(x_(6)), this->imu_acceleration * cos(x_(6)), this->odom_velocity * sin(x_(6)), this->imu_acceleration * sin(x_(6)), this->imu_yaw_rate;
    //Update();
}

void EKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    imu_acceleration_x = msg->linear_acceleration.x;
    imu_acceleration_y = msg->linear_acceleration.y;
    imu_acceleration = sqrt(pow(msg->linear_acceleration.x, 2) + pow(msg->linear_acceleration.y, 2));
    imu_yaw_rate = msg->angular_velocity.z;

    Predict(0.005);
    z_ << this->odom_velocity * cos(x_(6)), this->imu_acceleration * cos(x_(6)), this->odom_velocity * sin(x_(6)), this->imu_acceleration * sin(x_(6)), this->imu_yaw_rate;

    if(!flag)
    {
        Update(this->H2_);
        //flag = true;
    }
    else
    {
        Update(this->H1_);
        flag = false;
    }
}

void EKFNode::timer_callback()
{
    if(!is_init)
    {
        Init();
    }
    else
    {
        //z_ << this->odom_velocity * cos(x_(6)), this->imu_acceleration * cos(x_(6)), this->odom_velocity * sin(x_(6)), this->imu_acceleration * sin(x_(6)), this->imu_yaw_rate;

        //Update();

        geometry_msgs::msg::PoseStamped ekf;    
        ekf.header.frame_id = "ekf_frame_";
        ekf.header.stamp = this->t;

        ekf.pose.position.x = x_(0);
        ekf.pose.position.y = x_(3);
        ekf.pose.position.z = 0.0;

        ekf.pose.orientation.x = 0.0;
        ekf.pose.orientation.y = 0.0;
        ekf.pose.orientation.z = sin(x_(6)/2.0);
        ekf.pose.orientation.w = cos(x_(6)/2.0);

        this->pub_ekf_->publish(ekf);

        ekfdifference = sqrt(pow((x_(0) - this->pose_x), 2) + pow((x_(3) - this->pose_y), 2));
        ekfthetadifference = sqrt(pow((sin(x_(6)/2.0) - this->pose_yaw), 2));

        imudifference = sqrt(pow((this->imu_pose_x - this->pose_x), 2) + pow((this->imu_pose_y - this->pose_y), 2));
        imuthetadifference = sqrt(pow((this->imu_theta - this->pose_yaw), 2));

        odomdifference = sqrt(pow((this->odom_pose_x - this->pose_x), 2) + pow((this->odom_pose_y - this->pose_y), 2));
        odomthetadifference = sqrt(pow((this->odom_theta - this->pose_yaw), 2));

        std::ofstream ekfFile("/home/louis/ekfdifference.txt", std::ios::app);
        ekfFile << ekfdifference << std::endl;
        ekfFile.close();

        std::ofstream imuFile("/home/louis/imudifference.txt", std::ios::app);
        imuFile << imudifference << std::endl;
        imuFile.close();

        std::ofstream odomFile("/home/louis/odomdifference.txt", std::ios::app);
        odomFile << odomdifference << std::endl;
        odomFile.close();

        std::ofstream ekfThetaFile("/home/louis/ekfthetadifference.txt", std::ios::app);
        ekfThetaFile << ekfthetadifference << std::endl;
        ekfThetaFile.close();

        std::ofstream imuThetaFile("/home/louis/imuthetadifference.txt", std::ios::app);
        imuThetaFile << imuthetadifference << std::endl;
        imuThetaFile.close();

        std::ofstream odomThetaFile("/home/louis/odomthetadifference.txt", std::ios::app);
        odomThetaFile << odomthetadifference << std::endl;
        odomThetaFile.close();

        std::ofstream OdomDataFile("/home/louis/ODOMData.txt", std::ios::app);
        OdomDataFile << "x:" << this->odom_pose_x << std::endl;
        OdomDataFile << "y:" << this->odom_pose_y << std::endl;
        OdomDataFile.close();   

        std::ofstream ImuDataFile("/home/louis/IMUData.txt", std::ios::app);
        ImuDataFile << "x:" << this->imu_pose_x << std::endl;
        ImuDataFile << "y:" << this->imu_pose_y << std::endl;
        ImuDataFile.close(); 

        std::ofstream EKFDataFile("/home/louis/EKFData.txt", std::ios::app);
        EKFDataFile << "x:" << x_(0) << std::endl;
        EKFDataFile << "y:" << x_(3) << std::endl;
        EKFDataFile.close();        

        std::ofstream RealDataFile("/home/louis/REALData.txt", std::ios::app);
        RealDataFile << "x:" << this->pose_x << std::endl;
        RealDataFile << "y:" << this->pose_y << std::endl;
        RealDataFile.close();                             
    }
}

void EKFNode::Init()
{
    x_ = Eigen::VectorXd(8);
    P_ = Eigen::MatrixXd(8,8);
    Q_ = Eigen::MatrixXd(8,8);
    F_ = Eigen::MatrixXd(8,8);
    H1_ = Eigen::MatrixXd(5,8);
    H2_ = Eigen::MatrixXd(5,8);
    R_ = Eigen::MatrixXd(5,5);
    S_ = Eigen::MatrixXd(5,5);
    y_ = Eigen::VectorXd(5);
    K_ = Eigen::MatrixXd(8,5);
    z_ = Eigen::VectorXd(5);
    J_ = Eigen::MatrixXd(8,8);
    I_ = Eigen::MatrixXd::Identity(8,8);

    dt = 0.01;

    /*x_ << this->pose_x, this->odom_velocity * cos(this->pose_yaw), this->imu_acceleration * cos(this->pose_yaw), 
    this->pose_y, this->odom_velocity * sin(this->pose_yaw), this->imu_acceleration * sin(this->pose_yaw), 
    this->pose_yaw, (this->odom_yaw_rate + this->imu_yaw_rate) / 2;*/

    x_ << this->pose_x, 0.0, 0.0, 
    this->pose_y, 0.0, 0.0, 
    this->pose_yaw, 0.0;

    P_ << 
    1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 5.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 10.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 10.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0;

    /*Q_ << 
    0.1, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.005, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.005;*/

    Q_ << 
    0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.01;

    H1_ << 
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    H2_ << 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    R_ << 
    0.000000001, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.000000001, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.000000001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.000000001, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.000000001;

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

    /*J_ <<
    1, cos(x_(6)) * dt, 0, 0, 0, 0, -(x_(1)) * sin(x_(6)) * dt, 0,
    0, 1, dt, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,

    0, sin(x_(6)) * dt, 0, 1, 0, 0, x_(1) * cos(x_(6)) * dt, 0,
    0, 0, 0, 0, 1, dt, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,

    0, 0, 0, 0, 0, 0, 1, dt,
    0, 0, 0, 0, 0, 0, 0, 1;*/

    J_ <<
    1, cos(x_(6)) * dt, 0.5 * pow(dt, 2), 0, -sin(x_(6)) * dt, 0, 0, 0,
    0, 1, dt, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,

    0, sin(x_(6)) * dt, 0, 1, cos(x_(6)) * dt, 0.5 * pow(dt, 2), 0, 0,
    0, 0, 0, 0, 1, dt, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,

    0, 0, 0, 0, 0, 0, 1, dt,
    0, 0, 0, 0, 0, 0, 0, 1;
    
    P_ = J_ * P_ * J_.transpose() + Q_;
}

void EKFNode::Update(Eigen::MatrixXd Hj_)
{
    y_ = z_ - Hj_ * x_;
    S_ = Hj_* P_ * Hj_.transpose() + R_;
    K_ = P_ * Hj_.transpose() * S_.inverse();

    x_ = x_ + K_* y_;
    //P_ = (I_ - K_ * H_) * P_ * (I_ - K_ * H_).transpose() + K_ * R_ * K_.transpose();
    P_ = (I_ - K_ * Hj_) * P_;
}
}