#pragma once
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include <DDA/DDA.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

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
private:
    std::string m_input_filepath;
    std::string m_sensor_name;
    std::string m_reflector_name;
    int m_num_cells;
    int m_num_measurements;
    float m_lambda;
    double m_prior;

    std::vector<double> m_concentration; //(num_cells);
    std::vector<double> m_measurements; //(num_measurements);
    std::vector<std::vector<double>> m_lengthRayInCell; //(num_measurements, num_cells);

    std::vector<std::vector<bool>> m_occupancy_map;
    float m_rayMarchResolution;
    glm::vec2 m_mapOrigin;

    rclcpp::Publisher<Marker>::SharedPtr m_markerPub;
    
    void runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, int ppmxm);

    uint index2Dto1D(const glm::ivec2& index)
    {
        int num_cells_x = m_occupancy_map.size();
        int num_cells_y = m_occupancy_map[0].size();
	    return index.x + index.y*num_cells_x;
    }
    
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

static std_msgs::msg::ColorRGBA makeColor(float r, float g, float b, float a)
{
    std_msgs::msg::ColorRGBA color;
    color.r = std::clamp(r, 0.0f, 1.0f);
    color.g = std::clamp(g, 0.0f, 1.0f);
    color.b = std::clamp(b, 0.0f, 1.0f);
    color.a = std::clamp(a, 0.0f, 1.0f);
    return color;
}

double lerp(double start, double end, double proportion)
{
    if (proportion < 0 || std::isnan(proportion))
        return start;

    return start + (end - start) * std::min(1.0, proportion);
}

enum valueColorMode
{
    Linear,
    Logarithmic
};
std_msgs::msg::ColorRGBA valueToColor(double val, double lowLimit, double highLimit, valueColorMode mode)
{
    double r, g, b;
    double range;
    if (mode == valueColorMode::Logarithmic)
    {
        val = std::log10(val);
        range = (std::log10(highLimit) - std::log10(lowLimit)) / 4;
        lowLimit = std::log10(lowLimit);
    }
    else
    {
        range = (highLimit - lowLimit) / 4;
    }

    if (val < lowLimit + range)
    {
        r = 0;
        g = lerp(0, 1, (val - lowLimit) / (range));
        b = 1;
    }
    else if (val < lowLimit + 2 * range)
    {
        r = 0;
        g = 1;
        b = lerp(1, 0, (val - (lowLimit + range)) / (range));
    }
    else if (val < lowLimit + 3 * range)
    {
        r = (val - (lowLimit + 2 * range)) / (range);
        g = 1;
        b = 0;
    }
    else
    {
        r = 1;
        g = lerp(1, 0, (val - (lowLimit + 3 * range)) / (range));
        b = 0;
    }
    return makeColor(r, g, b, 1);
}