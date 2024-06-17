#include "ekf/ekf_node.hpp"

namespace ekf{

EKFNode::EKFNode() : Node("ekf_node")
{
    Ground_Truth_Subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    Odometrie_Subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 2, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    IMU_Subscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10).best_effort(), [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->imu_callback(msg);});
    EKF_Publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("ekf", 10);
}

void EKFNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Ground_Truth_x = msg->pose.position.x;
    Ground_Truth_y = msg->pose.position.y;
    Ground_Truth_Theta = msg->pose.orientation.z;
}

void EKFNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    t = msg->header.stamp;    
    Odom_Geschwindigkeit_x = msg->twist.twist.linear.x;
    Odom_Geschwindigkeit_y = msg->twist.twist.linear.y;
    Odom_Geschwindigkeit = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    Odometrie_Drehwinkelgeschwindigkeit = msg->twist.twist.angular.z;

    flag = true;
}

void EKFNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    IMU_Beschleunigung_x = msg->linear_acceleration.x;
    IMU_Beschleunigung_y = msg->linear_acceleration.y;
    IMU_Beschleunigung = sqrt(pow(msg->linear_acceleration.x, 2) + pow(msg->linear_acceleration.y, 2));
    IMU_Drehwinkelgeschwindigkeit = msg->angular_velocity.z;

    Praediktion(0.005);
    z << Odom_Geschwindigkeit * cos(x(6)), IMU_Beschleunigung * cos(x(6)), Odom_Geschwindigkeit * sin(x(6)), IMU_Beschleunigung * sin(x(6)), IMU_Drehwinkelgeschwindigkeit;

    if(!flag)
    {
        Korrektur(this->H2);
    }
    else
    {
        Korrektur(this->H1);
        geometry_msgs::msg::PoseStamped EKF_Nachricht;    
        EKF_Nachricht.header.frame_id = "EKF_Nachricht_frame";
        EKF_Nachricht.header.stamp = this->t;

        EKF_Nachricht.pose.position.x = x(0);
        EKF_Nachricht.pose.position.y = x(3);
        EKF_Nachricht.pose.position.z = 0.0;

        EKF_Nachricht.pose.orientation.x = 0.0;
        EKF_Nachricht.pose.orientation.y = 0.0;
        EKF_Nachricht.pose.orientation.z = sin(x(6)/2.0);
        EKF_Nachricht.pose.orientation.w = cos(x(6)/2.0);

        this->EKF_Publisher->publish(EKF_Nachricht);

        EKF_Differenz = sqrt(pow((x(0) - this->Ground_Truth_x), 2) + pow((x(3) - this->Ground_Truth_y), 2));
        EKF_Theta_Differenz = fabs(fmod(fmod(sin(x(6)/2.0) - this->Ground_Truth_Theta + 1, 2) + 2, 2) - 1);

        std::ofstream ekfFile("/home/louis/ekfdifference.txt", std::ios::app);
        ekfFile << EKF_Differenz << std::endl;
        ekfFile.close();

        std::ofstream ekfThetaFile("/home/louis/ekftheta.txt", std::ios::app);
        ekfThetaFile << EKF_Theta_Differenz << std::endl;
        ekfThetaFile.close();

        std::ofstream EKFDataFile("/home/louis/EKFData.txt", std::ios::app);
        EKFDataFile << "x:" << x(0) << std::endl;
        EKFDataFile << "y:" << x(3) << std::endl;
        EKFDataFile.close();        

        std::ofstream RealDataFile("/home/louis/REALData.txt", std::ios::app);
        RealDataFile << "x:" << this->Ground_Truth_x << std::endl;
        RealDataFile << "y:" << this->Ground_Truth_y << std::endl;
        RealDataFile.close();  

        flag = false;
    }
}

void EKFNode::Init()
{
    x = Eigen::VectorXd(8);
    P = Eigen::MatrixXd(8,8);
    Q = Eigen::MatrixXd(8,8);
    F = Eigen::MatrixXd(8,8);
    H1 = Eigen::MatrixXd(5,8);
    H2 = Eigen::MatrixXd(5,8);
    R = Eigen::MatrixXd(5,5);
    S = Eigen::MatrixXd(5,5);
    y = Eigen::VectorXd(5);
    K = Eigen::MatrixXd(8,5);
    z = Eigen::VectorXd(5);
    J = Eigen::MatrixXd(8,8);
    I = Eigen::MatrixXd::Identity(8,8);

    x << this->Ground_Truth_x, 0.0, 0.0, 
    this->Ground_Truth_y, 0.0, 0.0, 
    this->Ground_Truth_Theta, 0.0;

    P << 
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

    Q << 
    0.05, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.025, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.005, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.025, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.5;

    H1 << 
    0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    H2 << 
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    R << 
    0.000000001, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.000000001, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.000000001, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.000000001, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.000000000001;

    is_init = true;
}

void EKFNode::Praediktion(double dt)
{
    if(!is_init)
    {
        Init();
    }
    F <<
    1.0, dt, 0.5 * pow(dt, 2), 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 1.0, dt, 0.0, 0.0, 0.0, 0.0, 0.0,
    0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 1.0, dt, 0.5 * pow(dt, 2), 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 1.0, dt, 0.0, 0.0,
    0.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0,

    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0, dt,
    0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0;

    x = F * x;

    J <<
    1, cos(x(6)) * dt, 0.5 * pow(dt, 2), 0, -sin(x(6)) * dt, 0, 0, 0,
    0, 1, dt, 0, 0, 0, 0, 0,
    0, 0, 1, 0, 0, 0, 0, 0,

    0, sin(x(6)) * dt, 0, 1, cos(x(6)) * dt, 0.5 * pow(dt, 2), 0, 0,
    0, 0, 0, 0, 1, dt, 0, 0,
    0, 0, 0, 0, 0, 1, 0, 0,

    0, 0, 0, 0, 0, 0, 1, dt,
    0, 0, 0, 0, 0, 0, 0, 1;
    
    P = J * P * J.transpose() + Q;
}

void EKFNode::Korrektur(Eigen::MatrixXd Hj)
{
    y = z - Hj * x;
    S = Hj * P * Hj.transpose() + R;
    K = P * Hj.transpose() * S.inverse();

    x = x + K * y;
    //P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();
    P = (I - K * Hj) * P;
}
}