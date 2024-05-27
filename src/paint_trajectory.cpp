#include <rclcpp/rclcpp.hpp>
#include <PoseJSON.hpp>
#include <fstream>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <glm/gtc/quaternion.hpp>
#include <Utils.hpp>

using Marker=visualization_msgs::msg::Marker;
using MarkerArray=visualization_msgs::msg::MarkerArray;

class TrajectoryPainter : public rclcpp::Node
{
public:
    rclcpp::Publisher<MarkerArray>::SharedPtr sensorPub;
    rclcpp::Publisher<Marker>::SharedPtr reflectorPub;
    rclcpp::Publisher<Marker>::SharedPtr raysPub;
    
    TrajectoryPainter(): Node("Trajectory")
    {
        reflectorPub = create_publisher<Marker>("/reflector_marker", rclcpp::QoS(1).reliable().transient_local());
        raysPub = create_publisher<Marker>("/rays_marker", rclcpp::QoS(1).reliable().transient_local());
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
            reflectorMarker.scale.x = 0.2;
            reflectorMarker.scale.y = 0.2;
            reflectorMarker.scale.z = 0.2;

            reflectorMarker.color.r = 0;
            reflectorMarker.color.g = 0.6;
            reflectorMarker.color.b = 1;
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

        Marker linesMarker;
        {
            linesMarker.header.frame_id = "map";
            linesMarker.header.stamp = now();
            linesMarker.ns = "arrow";
            linesMarker.type = Marker::LINE_LIST;
            linesMarker.action = Marker::ADD;
            linesMarker.scale.x = 0.2;
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
            sensor.pose.position.z = 0;
            geometry_msgs::msg::PoseStamped reflector = mqtt_serialization::pose_from_json(json["reflectorTF"]);
            reflector.pose.position.z = 0;
            auto tdlas = mqtt_serialization::Utils::jsonToTDLAS(json["reading"]);

            reflectorMarker.points.push_back(reflector.pose.position);

            //sensor arrow
            {
                glm::quat cameraRotation = geoMsgToGLM(sensor.pose.orientation);
                glm::quat fixedRotation(glm::vec3{0, M_PI/2, 0});
                glm::quat combinedRotation = glm::cross(cameraRotation, fixedRotation);
                sensor.pose.orientation = glmToGeoMsg(combinedRotation);
                arrowMarker.pose = sensor.pose;
                arrowMarker.id = markerId++;
                sensorMarker.markers.push_back(arrowMarker);
            }

            //ray line
            {
                linesMarker.points.push_back(sensor.pose.position);
                linesMarker.points.push_back(reflector.pose.position);
                std_msgs::msg::ColorRGBA lineColor;
                lineColor.r = 1;
                lineColor.g = 0;
                lineColor.b = 0;
                lineColor.a = tdlas.average_ppmxm / 200.0;
                //color is pushed twice because it will do a gradient from point a to point b
                linesMarker.colors.push_back(lineColor);
                linesMarker.colors.push_back(lineColor);
            }

            numLines++;
        }

        //RCLCPP_INFO(get_logger(), "Number of lines parsed: %u", numLines);
        reflectorPub->publish(reflectorMarker);
        sensorPub->publish(sensorMarker);
        raysPub->publish(linesMarker);
    }

    glm::quat geoMsgToGLM(const geometry_msgs::msg::Quaternion& q)
    {
        return glm::quat(q.w, q.x, q. y, q.z);
    }
    geometry_msgs::msg::Quaternion glmToGeoMsg(const glm::quat& q)
    {
        geometry_msgs::msg::Quaternion qat;
        qat.x = q.x;
        qat.y = q.y;
        qat.z = q.z;
        qat.w = q.w;
        return qat;
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