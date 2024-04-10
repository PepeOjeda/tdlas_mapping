#include <rclcpp/rclcpp.hpp>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tdlas_mapping/BufferWrapper.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <tdlas_mapping/common.h>
#include <PoseJSON.hpp>

using TDLAS=olfaction_msgs::msg::TDLAS;
using KeyValue=diagnostic_msgs::msg::KeyValue;


class LogMeasurements : public rclcpp::Node
{
public:
    rclcpp::Subscription<TDLAS>::SharedPtr tdlasSub;
    rclcpp::Subscription<KeyValue>::SharedPtr followerPositionSub;
    BufferWrapper tf_buffer;

    std::ofstream file;
    LogMeasurements() : Node("log_measurements"), tf_buffer(get_clock())
    {
        tdlasSub = create_subscription<TDLAS>("/falcon/reading", 1, std::bind(&LogMeasurements::TDLAS_callback, this, std::placeholders::_1));
        followerPositionSub = create_subscription<KeyValue>("/mqtt2ros", 1, std::bind(&LogMeasurements::MQTT_callback, this, std::placeholders::_1));

        auto filepath = declare_parameter<std::string>("file_path", "measurement_log");
        file.open(filepath);
        RCLCPP_INFO(get_logger(), "Opened file: %s", filepath.c_str());
    }

    ~LogMeasurements()
    {
        file.close();
    }

    void checkAndWrite()
    {
        static builtin_interfaces::msg::Time stamp_last_msg;
        if(stamp_last_msg == last_tdlas_msg.header.stamp)
            return;

        stamp_last_msg = last_tdlas_msg.header.stamp;


        nlohmann::json json_entry;
        
        geometry_msgs::msg::PoseStamped rhodon_pose;
        try
        {
            tf2::Transform rhodon_tf;
            auto rhodon_tf_stamped = tf_buffer.buffer.lookupTransform("map", "camera", tf2::TimePointZero);
            tf2::fromMsg(rhodon_tf_stamped.transform, rhodon_tf);
            rhodon_pose.pose = transformToPose(rhodon_tf);
            rhodon_pose.header.stamp = now();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
            return;
        }
        
        json_entry["rhodon"] = nav2MQTT::to_json(rhodon_pose);
        json_entry["giraff"] = nav2MQTT::to_json(giraff_pose);
        json_entry["reading"] = tdlasToJson(last_tdlas_msg);
        file << json_entry.dump();
        file << "\n"; //entry delimiter for easier parsing
        RCLCPP_INFO(get_logger(), "Entry written to file");
    }

    TDLAS last_tdlas_msg;
    void TDLAS_callback(TDLAS::SharedPtr msg)
    {
        last_tdlas_msg = *msg;
    }

    geometry_msgs::msg::PoseStamped giraff_pose;
    void MQTT_callback(KeyValue::SharedPtr msg)
    {
        if(msg->key == "/giraff/status")
        {
            auto transform = parse_status_publisher_msg(msg->value);
            giraff_pose.pose = transformToPose(transform);
            giraff_pose.header.stamp = now();
        }
    }

    nlohmann::json tdlasToJson(const TDLAS& tdlas)
    {
        nlohmann::json json;
        json["average_ppmxm"] = tdlas.average_ppmxm;
        json["average_reflection_strength"] = tdlas.average_reflection_strength;
        json["average_absorption_strength"] = tdlas.average_absorption_strength;
        return json;
    }
};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<LogMeasurements>();
    rclcpp::Rate rate(10);
    while(rclcpp::ok())
    {
        node->checkAndWrite();
        rclcpp::spin_some(node);
        rate.sleep();
    }
}