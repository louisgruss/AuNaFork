#include "odom/odom_node.hpp"


OdomNode::OdomNode() : Node("odom_node")
{
    Odometrie_Subscriber = this->create_subscription<nav_msgs::msg::Odometry>("odom", 10, [this](const nav_msgs::msg::Odometry::SharedPtr msg){this->odom_callback(msg);});
    Ground_Truth_Subscriber = this->create_subscription<geometry_msgs::msg::PoseStamped>("localization_pose", 2, [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg){this->pose_callback(msg);});
    Odometrie_Position_Publisher = this->create_publisher<geometry_msgs::msg::PoseStamped>("Odometrie_Schaetzung", 10);
}

void OdomNode::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    Odometrie_Geschwindigkeit = sqrt(pow(msg->twist.twist.linear.x, 2) + pow(msg->twist.twist.linear.y, 2));
    Odometrie_Drehwinkelgeschwindigkeit = msg->twist.twist.angular.z;

    if(Vorherige_Odometrienachricht == nullptr)
    {
        Vorherige_Odometrienachricht = msg;
        position_x = this->Ground_Truth_x;
        position_y = this->Ground_Truth_y;
        Theta = this->Ground_Truth_Theta;
        delta = 0;
    }
    else
    {
        //delta = (msg->header.stamp.sec - Vorherige_Odometrienachricht->header.stamp.sec + (msg->header.stamp.nanosec - Vorherige_Odometrienachricht->header.stamp.nanosec) / 1000000000.0);
        auto dt = rclcpp::Time(msg->header.stamp) - rclcpp::Time(Vorherige_Odometrienachricht->header.stamp);
        delta = dt.seconds();
        Vorherige_Odometrienachricht = msg;
        position_x += Odometrie_Geschwindigkeit * cos(Theta) * delta;
        position_y += Odometrie_Geschwindigkeit * sin(Theta) * delta;
        Theta += Odometrie_Drehwinkelgeschwindigkeit * delta;
    }
    geometry_msgs::msg::PoseStamped Odometrie_Nachricht;
    Odometrie_Nachricht.header.frame_id = "Odometrie_Nachricht_frame";
    Odometrie_Nachricht.header.stamp = msg->header.stamp;
    //pred_pose.child_frame_id = base_frame_;

    Odometrie_Nachricht.pose.position.x = this->position_x;
    Odometrie_Nachricht.pose.position.y = this->position_y;
    Odometrie_Nachricht.pose.position.z = 0.0;

    Odometrie_Nachricht.pose.orientation.x = 0.0;
    Odometrie_Nachricht.pose.orientation.y = 0.0;
    Odometrie_Nachricht.pose.orientation.z = sin(Theta/2.0);
    Odometrie_Nachricht.pose.orientation.w = cos(Theta/2.0);

    this->Odometrie_Position_Publisher->publish(Odometrie_Nachricht);

    Odom_Differenz = sqrt(pow((this->position_x - this->Ground_Truth_x), 2) + pow((this->position_y - this->Ground_Truth_y), 2));
    Odometrie_Theta_Differenz = fabs(fmod(fmod(Theta - this->Ground_Truth_Theta + 1, 2) + 2, 2) - 1);

    std::ofstream OdomDataFile("/home/louis/ODOMData.txt", std::ios::app);
    OdomDataFile << "x:" << this->position_x << std::endl;
    OdomDataFile << "y:" << this->position_y << std::endl;
    OdomDataFile.close();  

    std::ofstream odomFile("/home/louis/odomdifference.txt", std::ios::app);
    odomFile << Odom_Differenz << std::endl;
    odomFile.close();

    std::ofstream odomThetaFile("/home/louis/odomtheta.txt", std::ios::app);
    odomThetaFile << Odometrie_Theta_Differenz << std::endl;
    odomThetaFile.close();
}

void OdomNode::pose_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    Ground_Truth_x = msg->pose.position.x;
    Ground_Truth_y = msg->pose.position.y;
    Ground_Truth_Theta = msg->pose.orientation.z;
}
