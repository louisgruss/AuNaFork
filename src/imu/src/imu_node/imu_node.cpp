#include "imu/imu_node.hpp"

ImuNode::ImuNode() : Node("imu_node")
{
    IMU_Subscriber = this->create_subscription<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(10).best_effort(), [this](const sensor_msgs::msg::Imu::SharedPtr msg){this->imu_callback(msg);});
    Ground_Truth_Subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    IMU_Position_Publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("IMU_Schaetzung", 10);
}

void ImuNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Ground_Truth_x = msg->pose.position.x;
    Ground_Truth_y = msg->pose.position.y;
    Ground_Truth_Theta = msg->pose.orientation.z;
}

void ImuNode::imu_callback(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    IMU_Beschleunigung_x = msg->linear_acceleration.x;
    IMU_Beschleunigung_y = msg->linear_acceleration.y; 
    IMU_Drehwinkelgeschwindigkeit = msg->angular_velocity.z;   

    if(Vorherige_IMU_Nachricht == nullptr)
    {
        position_x = this->Ground_Truth_x;
        position_y = this->Ground_Truth_y;
        Theta = this->Ground_Truth_Theta;
        Geschwindigkeit_x = 0.0;
        Geschwindigkeit_y = 0.0;
        Gesamtgeschwindigkeit = 0.0;
        Vorherige_IMU_Nachricht = msg;
    }
    else
    {
        //delta = (msg->header.stamp.sec - Vorherige_IMU_Nachricht->header.stamp.sec + (msg->header.stamp.nanosec - Vorherige_IMU_Nachricht->header.stamp.nanosec) / 1000000000.0); 
        auto dt = rclcpp::Time(msg->header.stamp) - rclcpp::Time(Vorherige_IMU_Nachricht->header.stamp);
        delta = dt.seconds();
        Geschwindigkeit_x += IMU_Beschleunigung_x * delta;
        Geschwindigkeit_y += IMU_Beschleunigung_y * delta;
        Gesamtgeschwindigkeit = sqrt(pow(Geschwindigkeit_x, 2) + pow(Geschwindigkeit_y, 2));
        position_x += Gesamtgeschwindigkeit * cos(Theta) * delta;
        position_y += Gesamtgeschwindigkeit * sin(Theta) * delta;
        Theta += IMU_Drehwinkelgeschwindigkeit * delta;
        Vorherige_IMU_Nachricht = msg;      
    }
    geometry_msgs::msg::PoseStamped IMU_Nachricht;    
    IMU_Nachricht.header.frame_id = "IMU_Nachricht_frame";
    IMU_Nachricht.header.stamp = msg->header.stamp;
    //pred_pose.child_frame_id = base_frame_;

    IMU_Nachricht.pose.position.x = position_x;
    IMU_Nachricht.pose.position.y = position_y;
    IMU_Nachricht.pose.position.z = 0.0;

    IMU_Nachricht.pose.orientation.x = 0.0;
    IMU_Nachricht.pose.orientation.y = 0.0;
    IMU_Nachricht.pose.orientation.z = sin(Theta/2.0);
    IMU_Nachricht.pose.orientation.w = cos(Theta/2.0);

    this->IMU_Position_Publisher->publish(IMU_Nachricht);

    IMU_Differenz = sqrt(pow((position_x - this->Ground_Truth_x), 2) + pow((position_y - this->Ground_Truth_y), 2));
    IMU_Theta_Differenz = fabs(fmod(fmod(Theta - this->Ground_Truth_x + 1, 2) + 2, 2) - 1);

    std::ofstream ImuDataFile("/home/louis/IMUData.txt", std::ios::app);
    ImuDataFile << "x:" << position_x << std::endl;
    ImuDataFile << "y:" << position_y << std::endl;
    ImuDataFile.close(); 

    std::ofstream imuFile("/home/louis/imudifference.txt", std::ios::app);
    imuFile << IMU_Differenz << std::endl;
    imuFile.close();

    std::ofstream imuThetaFile("/home/louis/imutheta.txt", std::ios::app);
    imuThetaFile << IMU_Theta_Differenz << std::endl;
    imuThetaFile.close();
}