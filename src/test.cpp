#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <nav2_msgs/action/navigate_to_pose.hpp>
#include <ament_imgui/ament_imgui.h>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <PoseJSON.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

using KeyValue=diagnostic_msgs::msg::KeyValue;
using NavToPose=nav2_msgs::action::NavigateToPose;
using PoseStamped=geometry_msgs::msg::PoseStamped;
using Marker=visualization_msgs::msg::Marker;

class Test: public rclcpp::Node
{
public:
    rclcpp_action::Client<NavToPose>::SharedPtr navToPoseClient1;
    rclcpp_action::Client<NavToPose>::SharedPtr navToPoseClient2;
    rclcpp::Publisher<KeyValue>::SharedPtr reactiveClient;

    tf2::Vector3 m_followerOffset;
    tf2::Vector3 m_reactiveGoalOffset;

    struct Markers{
        rclcpp::Publisher<Marker>::SharedPtr masterMarkerPub;
        rclcpp::Publisher<Marker>::SharedPtr followerMarkerPub;
        rclcpp::Publisher<Marker>::SharedPtr goalMarkerPub;
        Marker masterMarker;
        Marker followerMarker;
        Marker goalMarker;

        Markers()
        {
            //master
            {
                masterMarker.header.frame_id="map";
                masterMarker.action=Marker::ADD;
                masterMarker.id=1;
                masterMarker.type = Marker::ARROW;

                masterMarker.scale.x = 0.5;
                masterMarker.scale.y = 0.05;
                masterMarker.scale.z = 0.1;

                masterMarker.color.r = 0;
                masterMarker.color.g = 1;
                masterMarker.color.b = 0;
                masterMarker.color.a = 1;
            }
        
            //follower
            {
                followerMarker.header.frame_id="map";
                followerMarker.action=Marker::ADD;
                followerMarker.id=0;
                followerMarker.type = Marker::SPHERE;
                followerMarker.scale.x = 0.1;
                followerMarker.scale.y = 0.1;
                followerMarker.scale.z = 0.1;

                followerMarker.color.r = 1;
                followerMarker.color.g = 0;
                followerMarker.color.b = 0;
                followerMarker.color.a = 1;
                followerMarker.lifetime.sec = 100;
            }

            //goal
            {
                goalMarker.header.frame_id="map";
                goalMarker.action=Marker::ADD;
                goalMarker.id=0;
                goalMarker.type = Marker::SPHERE;
                goalMarker.scale.x = 0.1;
                goalMarker.scale.y = 0.1;
                goalMarker.scale.z = 0.1;

                goalMarker.color.r = 1;
                goalMarker.color.g = 1;
                goalMarker.color.b = 0;
                goalMarker.color.a = 1;
                goalMarker.lifetime.sec = 100;
            }
        }
    } markers;

    rclcpp::Subscription<PoseStamped>::SharedPtr rvizGoalSub;
    PoseStamped m_nextPosition;

    Test() : Node("Test")
    {
        navToPoseClient1 = rclcpp_action::create_client<NavToPose>(this, "/rhodon/nav2MQTT");
        navToPoseClient2 = rclcpp_action::create_client<NavToPose>(this, "/giraff/nav2MQTT");
        
        reactiveClient = create_publisher<KeyValue>("/ros2mqtt", 1);

        rvizGoalSub = create_subscription<PoseStamped>("/goal_pose", 1, std::bind(&Test::rvizGoalCallback, this, std::placeholders::_1) );

        using namespace std::chrono_literals;
        while(rclcpp::ok() && !navToPoseClient1->wait_for_action_server(5s))
                RCLCPP_INFO(get_logger(), "WAITING FOR NAV 1");
        while(rclcpp::ok() && !navToPoseClient2->wait_for_action_server(5s))
                RCLCPP_INFO(get_logger(), "WAITING FOR NAV 2");

        markers.masterMarkerPub = create_publisher<Marker>("/master_target", 1);
        markers.followerMarkerPub = create_publisher<Marker>("/follower_target", 1);
        markers.goalMarkerPub = create_publisher<Marker>("/goal_reactive", 1);
        
        float followerDistance = declare_parameter<float>("followerDistance", 1);
        m_followerOffset = {0, followerDistance, 0};

        float reactiveDistance = declare_parameter<float>("reactiveDistance", 1);
        m_reactiveGoalOffset = {reactiveDistance, 0, 0};
    }

    rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr sendGoal(rclcpp_action::Client<NavToPose>::SharedPtr client, const PoseStamped& goal_pose)
    {
        NavToPose::Goal goal;
        goal.pose = goal_pose;
        
        
        rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr goal_handle{nullptr};
        bool accepted = false;
        while(rclcpp::ok() && !accepted)
        {
            RCLCPP_INFO(get_logger(), "Sending..");
            auto future = client->async_send_goal(goal);
            rclcpp::spin_until_future_complete(shared_from_this(), future);

            goal_handle = future.get(); 
            accepted = goal_handle && goal_handle->get_status() == rclcpp_action::GoalStatus::STATUS_ACCEPTED;
            if(goal_handle)
                RCLCPP_INFO(get_logger(), "Status: %i, expected 1", goal_handle->get_status());
            else
                RCLCPP_INFO(get_logger(), "Future completed but goal_handle is null, trying again");
        }

        return goal_handle;
    }

    void run()
    {
        RCLCPP_WARN(shared_from_this()->get_logger(), "SENDING FIRST GOAL");
        sendGoal(navToPoseClient1, m_nextPosition);

        RCLCPP_WARN(shared_from_this()->get_logger(), "SENDING SECOND GOAL");
        sendGoal(navToPoseClient2, getOffsetFromMaster(m_followerOffset));

        RCLCPP_INFO(get_logger(), "GOAL DONE!");
    }

    void reactive()
    {
        navToPoseClient1->async_cancel_all_goals();
        navToPoseClient2->async_cancel_all_goals();
        RCLCPP_INFO(get_logger(), "STARTING REACTIVE");
        {
            PoseStamped reactiveGoal = getOffsetFromMaster(m_reactiveGoalOffset);

            KeyValue msg;
            msg.key = "/reactive";
            msg.value = nav2MQTT::to_json(reactiveGoal).dump();
            reactiveClient->publish(msg);
            //blockUntilGoalComplete(reactiveClient, goal_reactive1);
            //RCLCPP_INFO(get_logger(), "REACTIVE DONE");
        }
    }

    void mainLoop()
    {
        // may throw ament_index_cpp::PackageNotFoundError exception
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("tdlas_mapping");
        AMENT_IMGUI::setup( (package_share_directory+"/test_imgui.ini").c_str() );
        rclcpp::Rate rate(30);
        while (rclcpp::ok())
        {
            AMENT_IMGUI::StartFrame();
            
            ImGui::Begin("Distances");
            {
                ImGui::InputDouble("Follower Distance", &m_followerOffset[1]);
                ImGui::InputDouble("Reactive Goal Distance", &m_reactiveGoalOffset[0]);
                markers.goalMarker.pose = getOffsetFromMaster(m_reactiveGoalOffset).pose;
            }
            ImGui::End();

            ImGui::Begin("Next pose");
            {
                ImGui::InputFloat3("Position", (float*) &m_nextPosition.pose.position);
                
                float yaw = getYaw(m_nextPosition.pose.orientation);
                ImGui::InputFloat("Yaw", &yaw);
                m_nextPosition.pose.orientation = tf2::toMsg(tf2::Quaternion( {0,0,1}, yaw ));

                markers.masterMarker.pose = m_nextPosition.pose;
                markers.followerMarker.pose = getOffsetFromMaster(m_followerOffset).pose;

            }
            ImGui::End();

            if(ImGui::Button("Start"))
                run();
            
            if (ImGui::Button("Reactive"))
                reactive();


            //markers
            {
                markers.masterMarkerPub->publish(markers.masterMarker);
                markers.followerMarkerPub->publish(markers.followerMarker);
                markers.goalMarkerPub->publish(markers.goalMarker);
            }

            rclcpp::spin_some(shared_from_this());
            rate.sleep();
            AMENT_IMGUI::Render();
        }
        AMENT_IMGUI::close();
    }


    void blockUntilGoalComplete(rclcpp_action::Client<NavToPose>::SharedPtr client, rclcpp_action::ClientGoalHandle<NavToPose>::SharedPtr handle)
    {
        if(rclcpp::ok())
        {
            RCLCPP_INFO(get_logger(), "BLOCKING");

            auto future = client->async_get_result(handle);
            rclcpp::spin_until_future_complete(shared_from_this(), future);
        }   
    }



    void rvizGoalCallback(PoseStamped::SharedPtr msg)
    {
        RCLCPP_INFO(get_logger(), "Updating target pose");
        m_nextPosition = *msg;
    }

    double getYaw(const geometry_msgs::msg::Quaternion& quat)
    {
        tf2::Quaternion tfquat;
        tf2::fromMsg(quat, tfquat);

        tf2::Matrix3x3 m(tfquat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);
        return yaw;
    }

    tf2::Transform next_target_as_tf()
    {
        tf2::Transform target_as_tf;
            auto& p = m_nextPosition.pose.position;
            target_as_tf.setOrigin({p.x, p.y, p.z});
            tf2::Quaternion rot;
            tf2::fromMsg(m_nextPosition.pose.orientation, rot);
            target_as_tf.setRotation(rot);
        return target_as_tf;
    }

    PoseStamped transformToPose(const tf2::Transform& tf)
    {
        PoseStamped pose;
        pose.header.frame_id = "map";
        pose.header.stamp = now();
        pose.pose.position.x = tf.getOrigin().x();
        pose.pose.position.y = tf.getOrigin().y();
        pose.pose.position.z = tf.getOrigin().z();

        pose.pose.orientation = tf2::toMsg(tf.getRotation());
        return pose;
    }

    PoseStamped getOffsetFromMaster(const tf2::Vector3& offset)
    {
        tf2::Transform target_as_tf = next_target_as_tf();
        tf2::Transform offsetTransform(tf2::Quaternion::getIdentity(), offset);
        return transformToPose(target_as_tf * offsetTransform);
    }
};


int main(int argc, char** argv)
{
    
    rclcpp::init(argc, argv);

    auto node = std::make_shared<Test>();
    
    node->mainLoop();

    return 0;
}