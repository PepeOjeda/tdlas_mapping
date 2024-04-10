#include <rclcpp/rclcpp.hpp>
#include <rosgraph_msgs/msg/clock.hpp>

int main(int argc, char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<rclcpp::Node>("clockpub");
    auto start = node->now();

    auto pub = node->create_publisher<rosgraph_msgs::msg::Clock>("/clock", 5);
    rclcpp::Rate rate(20);
    while(rclcpp::ok())
    {
        rosgraph_msgs::msg::Clock msg;
        rclcpp::Duration ellapsed = (node->now()-start);
        msg.clock = rclcpp::Time{ellapsed.nanoseconds()};
        pub->publish(msg);
        rate.sleep();
    }
}