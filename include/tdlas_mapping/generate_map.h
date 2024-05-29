#pragma once
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tdlas_mapping/common.h>
#include <glm/vec2.hpp>
#include <tdlas_mapping/MapGenerator.hpp>


using Marker = visualization_msgs::msg::Marker;
class GenerateMapNode : public rclcpp::Node
{
public:
    GenerateMapNode();
    void readFile();
    void solve();
    void getEnvironment();
    void publishMarkers();
    void renderGUI();
private:
    std::optional<MapGenerator> mapGenerator;
    std::vector<double> m_concentration;

    std::string m_input_filepath;
    std::string m_sensor_name;
    std::string m_reflector_name;
    int m_num_cells;
    int m_num_measurements;
    float m_lambda;
    bool m_useRayPrior;
    double m_prior;

    std::vector<std::vector<bool>> m_occupancy_map;
    float m_rayMarchResolution;
    glm::vec2 m_mapOrigin;

    rclcpp::Publisher<Marker>::SharedPtr m_markerPub;
    glm::vec2 m_markerColorLimits;
    float m_markerAlpha = 0.5;

    uint index2Dto1D(const glm::ivec2& index)
    {
        int num_cells_x = m_occupancy_map.size();
        int num_cells_y = m_occupancy_map[0].size();
	    return index.x + index.y*num_cells_x;
    }  
};

