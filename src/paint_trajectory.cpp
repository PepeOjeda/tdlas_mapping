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
    rclcpp::Publisher<MarkerArray>::SharedPtr rhodonPub;
    rclcpp::Publisher<Marker>::SharedPtr giraffPub;
    
    TrajectoryPainter(): Node("Trajectory")
    {
        giraffPub = create_publisher<Marker>("/giraff_marker", rclcpp::QoS(1).reliable().transient_local());
        rhodonPub = create_publisher<MarkerArray>("/rhodon_marker", rclcpp::QoS(1).reliable().transient_local());
        declare_parameter<std::string>("filepath");
    }


    void readFileAndCreateMarkers()
    {
        MarkerArray rhodonMarker;
        Marker giraffMarker;
        {
            giraffMarker.header.frame_id="map";
            giraffMarker.action=Marker::ADD;
            giraffMarker.id=0;
            giraffMarker.type = Marker::POINTS;
            giraffMarker.scale.x = 0.1;
            giraffMarker.scale.y = 0.1;
            giraffMarker.scale.z = 0.1;

            giraffMarker.color.r = 1;
            giraffMarker.color.g = 0;
            giraffMarker.color.b = 0;
            giraffMarker.color.a = 1;
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
        std::string line;
        while(std::getline(file, line))

        {
            auto json = nlohmann::json::parse(line);
            geometry_msgs::msg::PoseStamped rhodon = nav2MQTT::from_json(json["rhodon"]);
            geometry_msgs::msg::PoseStamped giraff = nav2MQTT::from_json(json["giraff"]);

            giraffMarker.points.push_back(giraff.pose.position);
            arrowMarker.pose = rhodon.pose;
            arrowMarker.id = markerId++;
            rhodonMarker.markers.push_back(arrowMarker);
        }

        //RCLCPP_INFO(get_logger(), "PUBLISHING MARKERS");
        giraffPub->publish(giraffMarker);
        rhodonPub->publish(rhodonMarker);
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