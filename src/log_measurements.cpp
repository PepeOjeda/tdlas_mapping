#include <rclcpp/rclcpp.hpp>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <diagnostic_msgs/msg/key_value.hpp>
#include <tdlas_mapping/BufferWrapper.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <fstream>
#include <tdlas_mapping/common.h>
#include <json/PoseJSON.hpp>
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
    std::string arucoFrame;

    float aimingThresholdPx;
    int targetPx_X;
    int targetPx_Y;

    std::ofstream file;
    LogMeasurements() : Node("log_measurements"), tf_buffer(get_clock())
    {
        auto tdlasTopic = declare_parameter<std::string>("tdlasTopic", "falcon/reading");
        tdlasSub = create_subscription<TDLAS>(tdlasTopic, 1, std::bind(&LogMeasurements::TDLAS_callback, this, std::placeholders::_1));
        
        auto arucoTopic = declare_parameter<std::string>("arucoTopic", "aruco/detections");
        arucoSub = create_subscription<MarkerDetection>(arucoTopic, 1, std::bind(&LogMeasurements::Aruco_callback, this, std::placeholders::_1));

        aimingThresholdPx = declare_parameter<float>("aimingThresholdPx", 50);
        targetPx_X = declare_parameter<int>("targetPx_X", 1920 / 2);
        targetPx_Y = declare_parameter<int>("targetPx_Y", 1080 / 2);

        fixedFrame = declare_parameter<std::string>("fixedFrame", "");
        sensorFrame = declare_parameter<std::string>("sensorFrame", "");
        reflectorFrame = declare_parameter<std::string>("reflectorFrame", "");
        arucoFrame = declare_parameter<std::string>("arucoFrame", "");
        
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
            float errorX = std::abs(last_aruco.camera_space_point.x - targetPx_X);
            float errorY = std::abs(last_aruco.camera_space_point.y - targetPx_Y);
            aimingAtAruco =  errorX < aimingThresholdPx 
                         &&  errorY < aimingThresholdPx;
            
            RCLCPP_WARN(get_logger(), "Aiming error: %.2f, %.2f px", errorX, errorY);
        }
        else
            RCLCPP_WARN(get_logger(), "We have not received aruco detections in a while.");

        geometry_msgs::msg::PoseStamped sensor_pose = getPoseFromTF(sensorFrame);
        geometry_msgs::msg::PoseStamped reflector_pose = getPoseFromTF(reflectorFrame);
        geometry_msgs::msg::PoseStamped aruco_pose = getPoseFromTF(arucoFrame);
        
        nlohmann::json json_entry;
        json_entry["sensorTF"] = jsonSerialization::pose_to_json(sensor_pose);
        json_entry["reflectorTF"] = jsonSerialization::pose_to_json(reflector_pose);
        json_entry["arucoTF"] = jsonSerialization::pose_to_json(aruco_pose);
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
        last_aruco.stamp = now();
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
            pose.header.frame_id = fixedFrame;
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