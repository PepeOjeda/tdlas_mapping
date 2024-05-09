#include <rclcpp/rclcpp.hpp>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tdlas_mapping/BufferWrapper.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <tdlas_mapping/common.h>
#include <PoseJSON.hpp>
#include <image_marker_msgs/msg/marker_detection.hpp>

using MarkerDetection=image_marker_msgs::msg::MarkerDetection;
using TDLAS=olfaction_msgs::msg::TDLAS;
using KeyValue=diagnostic_msgs::msg::KeyValue;


class LogMeasurements : public rclcpp::Node
{
public:
    rclcpp::Subscription<TDLAS>::SharedPtr tdlasSub;
    rclcpp::Subscription<MarkerDetection>::SharedPtr arucoSub;
    BufferWrapper tf_buffer;

    std::string fixedFrame;
    std::string sensorFrame;
    std::string reflectorFrame;

    float aimingThresholdPx;

    std::ofstream file;
    LogMeasurements() : Node("log_measurements"), tf_buffer(get_clock())
    {
        auto tdlasTopic = declare_parameter<std::string>("tdlasTopic", "falcon/reading");
        tdlasSub = create_subscription<TDLAS>(tdlasTopic, 1, std::bind(&LogMeasurements::TDLAS_callback, this, std::placeholders::_1));
        
        auto arucoTopic = declare_parameter<std::string>("arucoTopic", "aruco/detections");
        arucoSub = create_subscription<MarkerDetection>(arucoTopic, 1, std::bind(&LogMeasurements::Aruco_callback, this, std::placeholders::_1));

        aimingThresholdPx = declare_parameter<float>("aimingThresholdPx", 50);

        fixedFrame = declare_parameter<std::string>("fixedFrame", "");
        sensorFrame = declare_parameter<std::string>("sensorFrame", "");
        reflectorFrame = declare_parameter<std::string>("reflectorFrame", "");
        
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

        bool aimingAtAruco = false;
        //if we have seen the aruco recently
        if((now()-last_aruco.stamp).seconds()<0.5)
        {
            aimingAtAruco = last_aruco.camera_space_point.x < aimingThresholdPx 
                         && last_aruco.camera_space_point.y < aimingThresholdPx;
        }

        geometry_msgs::msg::PoseStamped sensor_pose = getPoseFromTF(sensorFrame);
        geometry_msgs::msg::PoseStamped reflector_pose = getPoseFromTF(reflectorFrame);
        geometry_msgs::msg::PoseStamped aruco_pose = getPoseFromTF(last_aruco.transform.child_frame_id);
        
        nlohmann::json json_entry;
        json_entry["sensorTF"] = nav2MQTT::to_json(sensor_pose);
        json_entry["reflectorTF"] = nav2MQTT::to_json(reflector_pose);
        json_entry["arucoTF"] = nav2MQTT::to_json(aruco_pose);
        json_entry["isAiming"] = aimingAtAruco;
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

    MarkerDetection last_aruco;
    void Aruco_callback(MarkerDetection::SharedPtr msg)
    {
        last_aruco = *msg;
    }

    nlohmann::json tdlasToJson(const TDLAS& tdlas)
    {
        nlohmann::json json;
        json["average_ppmxm"] = tdlas.average_ppmxm;
        json["average_reflection_strength"] = tdlas.average_reflection_strength;
        json["average_absorption_strength"] = tdlas.average_absorption_strength;
        return json;
    }

    geometry_msgs::msg::PoseStamped getPoseFromTF(std::string frame)
    {
        geometry_msgs::msg::PoseStamped pose;
        try
        {
            tf2::Transform tf;
            auto tf_stamped = tf_buffer.buffer.lookupTransform(fixedFrame, frame, tf2::TimePointZero);
            tf2::fromMsg(tf_stamped.transform, tf);
            pose.pose = transformToPose(tf);
            pose.header.stamp = now();
        }
        catch(const std::exception& e)
        {
            RCLCPP_ERROR(get_logger(), "Error getting %s TF:\n %s", frame.c_str(), e.what());
        }
        return pose;
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