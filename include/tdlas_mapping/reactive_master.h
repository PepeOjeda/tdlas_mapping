#pragma once
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <tf2/transform_datatypes.h>
#include <tdlas_mapping/BufferWrapper.h>
#include <geometry_msgs/msg/twist.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tdlas_mapping/PID.h>
#include <diagnostic_msgs/msg/key_value.hpp>

using GoToPose = nav2_msgs::action::NavigateToPose;

class ReactiveMaster
{
    using Twist = geometry_msgs::msg::Twist;
    using Marker = visualization_msgs::msg::Marker;
public:
    ReactiveMaster(std::shared_ptr<rclcpp::Node> n);
    ~ReactiveMaster();
    void run(const geometry_msgs::msg::PoseStamped& goal_pose);
    void render();

private:
    std::shared_ptr<rclcpp::Node> node;
    BufferWrapper tf_buffer;
    std::unique_ptr<PID> pid;

    double m_linearSpeed;
    double m_stoppingDistance;
    double m_directionTolerance;
    std::string m_localFrame;
    tf2::Transform m_currentTransform;

    rclcpp::Publisher<Twist>::SharedPtr cmdPub;
    rclcpp::Publisher<diagnostic_msgs::msg::KeyValue>::SharedPtr runningPub;
    rclcpp::Publisher<Marker>::SharedPtr targetMarkerPub;
    rclcpp::Publisher<Marker>::SharedPtr arrowMarkerPub;

    void updateTFs();

};