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
    BufferWrapper tf_buffer;

    std::string fixedFrame;
    std::string sensorFrame;
    std::string reflectorFrame;

    std::ofstream file;
    LogMeasurements() : Node("log_measurements"), tf_buffer(get_clock())
    {
        tdlasSub = create_subscription<TDLAS>("/falcon/reading", 1, std::bind(&LogMeasurements::TDLAS_callback, this, std::placeholders::_1));

        auto filepath = declare_parameter<std::string>("file_path", "measurement_log");

        fixedFrame = declare_parameter<std::string>("fixedFrame", "");
        sensorFrame = declare_parameter<std::string>("sensorFrame", "");
        reflectorFrame = declare_parameter<std::string>("reflectorFrame", "");
        
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
        
        geometry_msgs::msg::PoseStamped sensor_pose;
        geometry_msgs::msg::PoseStamped reflector_pose;
        try
        {
            tf2::Transform sensor_tf;
            auto sensor_tf_stamped = tf_buffer.buffer.lookupTransform(fixedFrame, sensorFrame, tf2::TimePointZero);
            tf2::fromMsg(sensor_tf_stamped.transform, sensor_tf);
            sensor_pose.pose = transformToPose(sensor_tf);
            sensor_pose.header.stamp = now();

            tf2::Transform reflector_tf;
            auto reflector_tf_stamped = tf_buffer.buffer.lookupTransform(fixedFrame, reflectorFrame, tf2::TimePointZero);
            tf2::fromMsg(reflector_tf_stamped.transform, reflector_tf);
            reflector_pose.pose = transformToPose(reflector_tf);
            reflector_pose.header.stamp = now();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "%s", e.what());
            return;
        }
        
        json_entry["sensor"] = nav2MQTT::to_json(sensor_pose);
        json_entry["reflector"] = nav2MQTT::to_json(reflector_pose);
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