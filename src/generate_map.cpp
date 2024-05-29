#include <tdlas_mapping/generate_map.h>
#include <fstream>
#include <json/PoseJSON.hpp>
#include <tdlas_mapping/common.h>
#include <json/Utils.hpp>
#include <tf2/LinearMath/Transform.h>

#include <fmt/core.h>
#include <filesystem>

#include <ament_imgui/ament_imgui.h>


using PoseStamped = geometry_msgs::msg::PoseStamped;
using TDLAS = olfaction_msgs::msg::TDLAS;
using namespace std::chrono_literals;

GenerateMapNode::GenerateMapNode() : Node("GenerateMapNode")
{
    m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.05f);
    m_lambda = declare_parameter<float>("lambda", 0.01);
    m_prior = declare_parameter<double>("prior", 0.0);
    m_useRayPrior = declare_parameter<bool>("useRayPrior", false);
    m_input_filepath = declare_parameter<std::string>("filepath", "measurement_log");
    m_sensor_name = declare_parameter<std::string>("sensor_name", "sensorTF");
    m_reflector_name = declare_parameter<std::string>("reflector_name", "reflectorTF");
    m_markerPub = create_publisher<Marker>("/concentration_markers", 1);
    m_markerColorLimits = glm::vec2(0, 10);
}

void GenerateMapNode::getEnvironment()
{
    // We are going to assume an empty map that's just the bounds of the measurements
    // for an example of how to actually do this for real, see the robot2023 branch
    std::ifstream file(m_input_filepath);
    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Could not open file %s", m_input_filepath.c_str());
        raise(SIGTRAP);
    }

    uint num_cells_x, num_cells_y;
    // find environment dimensions (Bounding box)
    glm::vec2 min{FLT_MAX, FLT_MAX}, max{-FLT_MAX, -FLT_MAX};
    std::string line;
    while (std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform sensor = poseToTransform(jsonSerialization::pose_from_json(json[m_sensor_name]).pose);
        tf2::Transform reflector = poseToTransform(jsonSerialization::pose_from_json(json[m_reflector_name]).pose);

        min.x = std::min({(double)min.x, sensor.getOrigin().x(), reflector.getOrigin().x()});
        min.y = std::min({(double)min.y, sensor.getOrigin().y(), reflector.getOrigin().y()});

        max.x = std::max({(double)max.x, sensor.getOrigin().x(), reflector.getOrigin().x()});
        max.y = std::max({(double)max.y, sensor.getOrigin().y(), reflector.getOrigin().y()});
    }

    // a little bit of arbitrary padding around the used area
    //max += glm::vec2{m_rayMarchResolution * 4, m_rayMarchResolution * 4};
    //min += glm::vec2{-m_rayMarchResolution * 4, -m_rayMarchResolution * 4};

    RCLCPP_INFO(get_logger(), "Environment bounds: (%.2f, %.2f) --- (%.2f, %.2f)", min.x, min.y, max.x, max.y);
    num_cells_x = std::ceil((max.x - min.x) / m_rayMarchResolution);
    num_cells_y = std::ceil((max.y - min.y) / m_rayMarchResolution);

    file.close();

    m_num_cells = num_cells_x * num_cells_y;

    m_mapOrigin.x = min.x;
    m_mapOrigin.y = min.y;

    m_occupancy_map.resize(num_cells_x, std::vector<bool>(num_cells_y, true));


    MapGeneratorOptions options;
    options.lambda = m_lambda;
    options.numCells = m_num_cells;
    options.prior = m_prior;
    options.useRayPrior = m_useRayPrior;
    mapGenerator.emplace(options, m_concentration);
    mapGenerator->SetMap(m_occupancy_map, m_mapOrigin, m_rayMarchResolution);
}

void GenerateMapNode::readFile()
{
    std::ifstream file(m_input_filepath);

    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Could not open file %s", m_input_filepath.c_str());
        raise(SIGTRAP);
    }

    std::vector<TDLASMeasurement> measurements;
    int measurementIndex = 0;
    std::string line;
    while (std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform sensor = poseToTransform(jsonSerialization::pose_from_json(json[m_sensor_name]).pose);
        tf2::Transform reflector = poseToTransform(jsonSerialization::pose_from_json(json[m_reflector_name]).pose);
        TDLAS tdlas = jsonSerialization::Utils::jsonToTDLAS(json["reading"]);

        measurements.push_back(TDLASMeasurement());
        measurements.back().ppmxm = tdlas.average_ppmxm;

        // fill in the cell raytracing thing
        measurements.back().origin = glm::fromTF(sensor.getOrigin());
        measurements.back().reflectorPosition = glm::fromTF(reflector.getOrigin());
        measurements.back().direction = glm::normalize(measurements.back().reflectorPosition - measurements.back().origin);

        measurementIndex++;
    }
    RCLCPP_INFO(get_logger(), "Number of lines parsed: %u", measurementIndex);
    file.close();

    mapGenerator->SetMeasurements(measurements);
}

class IterationCallback : public ceres::IterationCallback
{
public:
    explicit IterationCallback(GenerateMapNode& _node, MapGenerator& _mapGenerator) : node(_node), mapGenerator(_mapGenerator)
    {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
    {
        mapGenerator.WriteHeatmap();
        node.publishMarkers();
        //mapGenerator.publishMarkers();
        if (rclcpp::ok())
            return ceres::SOLVER_CONTINUE;
        else
            return ceres::SOLVER_ABORT;
    }

private:
    GenerateMapNode& node;
    MapGenerator& mapGenerator;
};

void GenerateMapNode::solve()
{
    mapGenerator->Solve({new IterationCallback(*this, *mapGenerator)});
}

void GenerateMapNode::publishMarkers()
{
    Marker marker;
    marker.header.frame_id = "map";
    marker.header.stamp = now();
    marker.type = Marker::POINTS;
    marker.scale.x = marker.scale.y = m_rayMarchResolution;

    for (int i = 0; i < m_occupancy_map.size(); i++)
    {
        for (int j = 0; j < m_occupancy_map[0].size(); j++)
        {
            if (m_occupancy_map[i][j])
            {
                marker.points.push_back(glm::toPoint(m_mapOrigin + glm::vec2(i, j) * m_rayMarchResolution));

                float ppmxm = m_concentration[i + j * m_occupancy_map.size()];
                std_msgs::msg::ColorRGBA color = valueToColor(ppmxm, m_markerColorLimits.x, m_markerColorLimits.y, valueColorMode::Linear);
                color.a = m_markerAlpha;
                marker.colors.push_back(color);
            }
        }
    }

    m_markerPub->publish(marker);
}

void GenerateMapNode::renderGUI()
{
    AmentImgui::Setup(nullptr, "TDLAS optimization", 250, 200);

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {

        AmentImgui::StartFrame();
        ImGui::Begin("Colors");
        ImGui::SetWindowSize(ImVec2(177, 100));
        ImGui::DragFloat("Min", &m_markerColorLimits.x, 0.1f, 0.0f, FLT_MAX, "%.1f");
        ImGui::DragFloat("Max", &m_markerColorLimits.y, 0.1f, 0.0f, FLT_MAX, "%.1f");
        ImGui::DragFloat("Alpha", &m_markerAlpha, 0.005f, 0.0f, 1.0f, "%.2f");
        ImGui::End();

        AmentImgui::Render();
        rate.sleep();
    }
    AmentImgui::Close();
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GenerateMapNode>();

    std::thread renderThread([node]() { node->renderGUI(); });

    node->getEnvironment();
    node->readFile();
    node->solve();

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        node->publishMarkers();
    }

    renderThread.join();
    rclcpp::shutdown();
    return 0;
}