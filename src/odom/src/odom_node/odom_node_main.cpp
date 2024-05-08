#include "odom/odom_node.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<OdomNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
