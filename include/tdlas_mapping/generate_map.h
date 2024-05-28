#pragma once
#include <rclcpp/rclcpp.hpp>
#include <DDA/DDA.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tdlas_mapping/common.h>

using Marker = visualization_msgs::msg::Marker;
class MapGenerator : public rclcpp::Node
{
public:
    MapGenerator();
    void readFile();
    void solve();
    void getEnvironment();
    void writeHeatmap();
    void publishMarkers();
    void renderGUI();
private:
    std::string m_input_filepath;
    std::string m_sensor_name;
    std::string m_reflector_name;
    int m_num_cells;
    int m_num_measurements;
    float m_lambda;
    bool m_useRayPrior;
    double m_prior;

    std::vector<double> m_concentration; //(num_cells);
    std::vector<RollingAverage> m_concentrationPrior; //(num_cells);
    std::vector<double> m_measurements; //(num_measurements);
    std::vector<std::vector<double>> m_lengthRayInCell; //(num_measurements, num_cells);

    std::vector<std::vector<bool>> m_occupancy_map;
    float m_rayMarchResolution;
    glm::vec2 m_mapOrigin;

    rclcpp::Publisher<Marker>::SharedPtr m_markerPub;
    glm::vec2 m_markerColorLimits;
    float m_markerAlpha = 0.5;
    
    void runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, int ppmxm);

    uint index2Dto1D(const glm::ivec2& index)
    {
        int num_cells_x = m_occupancy_map.size();
        int num_cells_y = m_occupancy_map[0].size();
	    return index.x + index.y*num_cells_x;
    }

    void addNoiseToSolution(float stdev);
    
};


namespace glm
{
    static glm::vec2 fromTF(const tf2::Vector3& v)
    {
        return glm::vec2(v.x(), v.y());
    }

    static geometry_msgs::msg::Point toPoint (const glm::vec2& vec)
    {
        geometry_msgs::msg::Point p;
        p.x = vec.x;
        p.y = vec.y;
        return p;
    }
}