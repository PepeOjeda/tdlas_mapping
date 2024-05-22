#include <rclcpp/rclcpp.hpp>
#include <PoseJSON.hpp>
#include <fstream>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

using Marker=visualization_msgs::msg::Marker;
using MarkerArray=visualization_msgs::msg::MarkerArray;

class TrajectoryPainter : public rclcpp::Node
{
public:
    rclcpp::Publisher<MarkerArray>::SharedPtr sensorPub;
    rclcpp::Publisher<Marker>::SharedPtr reflectorPub;
    
    TrajectoryPainter(): Node("Trajectory")
    {
        reflectorPub = create_publisher<Marker>("/reflector_marker", rclcpp::QoS(1).reliable().transient_local());
        sensorPub = create_publisher<MarkerArray>("/sensor_marker", rclcpp::QoS(1).reliable().transient_local());
        declare_parameter<std::string>("filepath");
    }


    void readFileAndCreateMarkers()
    {
        MarkerArray sensorMarker;
        Marker reflectorMarker;
        {
            reflectorMarker.header.frame_id="map";
            reflectorMarker.action=Marker::ADD;
            reflectorMarker.id=0;
            reflectorMarker.type = Marker::POINTS;
            reflectorMarker.scale.x = 0.1;
            reflectorMarker.scale.y = 0.1;
            reflectorMarker.scale.z = 0.1;

            reflectorMarker.color.r = 1;
            reflectorMarker.color.g = 0;
            reflectorMarker.color.b = 0;
            reflectorMarker.color.a = 1;
        }


        Marker arrowMarker;
        {
            arrowMarker.header.frame_id = "map";
            arrowMarker.header.stamp = now();
            arrowMarker.ns = "arrow";
            arrowMarker.type = Marker::ARROW;
            arrowMarker.action = Marker::ADD;
            arrowMarker.scale.x = 0.1;
            arrowMarker.scale.y = 0.03;
            arrowMarker.scale.z = 0.05;
            arrowMarker.color.a = 1;
            arrowMarker.color.r = 0;
            arrowMarker.color.g = 1;
            arrowMarker.color.b = 0;
        }

        std::string filepath = get_parameter("filepath").as_string();
        std::ifstream file (filepath);
        if(!file.is_open())
        {
            RCLCPP_ERROR(get_logger(), "Could not open file: %s", filepath.c_str());
            return;
        }
        
        
        int markerId = 0;
        uint numLines = 0;
        std::string line;
        while(std::getline(file, line))
        {
            auto json = nlohmann::json::parse(line);
            geometry_msgs::msg::PoseStamped sensor = mqtt_serialization::pose_from_json(json["sensorTF"]);
            geometry_msgs::msg::PoseStamped reflector = mqtt_serialization::pose_from_json(json["reflectorTF"]);

            reflectorMarker.points.push_back(reflector.pose.position);
            arrowMarker.pose = sensor.pose;
            arrowMarker.id = markerId++;
            sensorMarker.markers.push_back(arrowMarker);
            numLines++;
        }

        RCLCPP_INFO(get_logger(), "Number of lines parsed: %u", numLines);
        reflectorPub->publish(reflectorMarker);
        sensorPub->publish(sensorMarker);
    }
};







int main (int argc, char** argv)
{
    rclcpp::init(argc,argv);

    auto node = std::make_shared<TrajectoryPainter>();

    rclcpp::Rate rate(1);
    while(rclcpp::ok())
    {
        node->readFileAndCreateMarkers();
        rate.sleep();
    }

    return 0;
}