#pragma once
#define EIGEN_INITIALIZE_MATRICES_BY_ZERO
#include <rclcpp/rclcpp.hpp>
#include <eigen3/Eigen/Core>
#include <DDA/DDA.h>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

class MapGenerator : public rclcpp::Node
{

public:
    MapGenerator();
    void readFile();
    void solve();
    void getEnvironment();
    void writeHeatmap();
private:
    std::string m_input_filepath;
    std::string m_sensor_name;
    std::string m_reflector_name;
    int m_num_cells;
    int m_num_measurements;
    float m_lambda;

    Eigen::VectorXf m_concentration; //(num_cells);
    Eigen::VectorXf m_measurements; //(num_measurements);
    Eigen::MatrixXf m_lengthRayInCell; //(num_measurements, num_cells);

    std::vector<std::vector<bool>> m_occupancy_map;
    float m_rayMarchResolution;
    glm::vec2 m_mapOrigin;
    
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
}