#include <tdlas_mapping/generate_map.h>
#include <fstream>
#include <PoseJSON.hpp>
#include <tdlas_mapping/common.h>
#include <olfaction_msgs/msg/tdlas.hpp>
#include <tf2/LinearMath/Transform.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fmt/core.h>
#include <filesystem>

#include <lsqcpp/lsqcpp.hpp> 

#define RAYS_IMAGE 1

#if RAYS_IMAGE
static cv::Mat rays_image;
#endif

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TDLAS=olfaction_msgs::msg::TDLAS;
using namespace std::chrono_literals;

static TDLAS jsonToTDLAS(const nlohmann::json& json)
{
    TDLAS tdlas;
    tdlas.average_ppmxm = (double) json["average_ppmxm"].get<int>();
    tdlas.average_absorption_strength = json["average_absorption_strength"].get<double>();
    tdlas.average_reflection_strength = json["average_reflection_strength"].get<double>();
    return tdlas;
}

MapGenerator::MapGenerator() : Node("MapGenerator")
{
    m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.05f);
    m_lambda = declare_parameter<float>("lambda", 0.01);
    m_input_filepath = declare_parameter<std::string>("filepath", "measurement_log");
    m_sensor_name = declare_parameter<std::string>("sensor_name", "sensorTF");
    m_reflector_name = declare_parameter<std::string>("reflector_name", "reflectorTF");

}

void MapGenerator::readFile()
{
    std::ifstream file(m_input_filepath);
    
    if(!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Could not open file %s", m_input_filepath.c_str());
        raise(SIGTRAP);
    }
    

    //Count the number of entries and size the matrices accordingly
    int numberOfMeasurements=0;
    std::string line;
    while( std::getline(file, line) )
        numberOfMeasurements++;
    
    m_measurements.resize(numberOfMeasurements);
    m_lengthRayInCell.resize(numberOfMeasurements, m_num_cells);
    m_concentration.resize(m_num_cells);
    m_lengthRayInCell.fill(m_lambda); //prior correction
    
    //restart the ifstream
    file.clear();
    file.seekg(0, std::ios::beg);
    
    int measurementIndex = 0;

#if RAYS_IMAGE
    rays_image.create(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC3);
    rays_image.setTo(cv::Vec3b(255,255,255));
#endif

    while(std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform sensor = poseToTransform( mqtt_serialization::pose_from_json(json[m_sensor_name]).pose );
        tf2::Transform reflector = poseToTransform( mqtt_serialization::pose_from_json(json[m_reflector_name]).pose );
        TDLAS tdlas = jsonToTDLAS(json["reading"]);

        m_measurements[measurementIndex] = tdlas.average_ppmxm;

        //fill in the cell raytracing thing
        glm::vec2 rayOrigin = glm::fromTF( sensor.getOrigin() );
        glm::vec2 rayDirection = glm::fromTF( tf2::quatRotate(sensor.getRotation(), {0,1,0}) ); 
        glm::vec2 reflectorPosition = glm::fromTF( reflector.getOrigin() );

        runDDA(rayOrigin, rayDirection, reflectorPosition, measurementIndex, tdlas.average_ppmxm);
        measurementIndex++;
    }
    file.close();

#if RAYS_IMAGE
    for(int i = 0; i<m_occupancy_map.size(); i++)
    {
        for(int j=0; j<m_occupancy_map[0].size(); j++)
        {
            if(!m_occupancy_map[i][j])
                rays_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0,0,0);
        }
    }
    cv::imwrite("rays.png", rays_image); 
#endif
}

struct ObjectiveFunction
{
    static constexpr bool ComputesJacobian = false;
    static inline Eigen::MatrixXf lengthInCell;
    static inline Eigen::VectorXf measurements;
    static inline float lambda;

    template<typename Scalar, int Inputs, int Outputs>
    void operator()(const Eigen::Matrix<Scalar, Inputs, 1> &xval,
                    Eigen::Matrix<Scalar, Outputs, 1> &fval) const
    {
        fval.resize(1);
        Eigen::VectorXf predictedReading = lengthInCell * xval;
        fval(0) = (measurements-predictedReading).norm();

        Eigen::Index rows = predictedReading.rows();
        for(Eigen::Index i = 0; i<rows; i++)
        {
            auto val = predictedReading(i);
            fval(0) += val * val * lambda;
            if(val < 0)
                fval(0) += 10000 * std::abs(val);
        }
    }
};

void MapGenerator::solve()
{
    ObjectiveFunction::lengthInCell = m_lengthRayInCell;
    ObjectiveFunction::lambda = m_lambda;
    ObjectiveFunction::measurements = m_measurements;
    
    lsqcpp::GaussNewtonX<float, ObjectiveFunction, lsqcpp::ArmijoBacktracking> optimizer;

    // Set it to 0 or negative for infinite iterations (default is 0).
    optimizer.setMaximumIterations(100);

    // The optimizer stops minimizing if the gradient length falls below this
    // value.
    // Set it to 0 or negative to disable this stop criterion (default is 1e-9).
    optimizer.setMinimumGradientLength(1e-6);

    // The optimizer stops minimizing if the step length falls below this
    // value.
    // Set it to 0 or negative to disable this stop criterion (default is 1e-9).
    optimizer.setMinimumStepLength(1e-6);

    // The optimizer stops minimizing if the error falls below this
    // value.
    // Set it to 0 or negative to disable this stop criterion (default is 0).
    optimizer.setMinimumError(0);

    // Set the parameters of the step refiner (Armijo Backtracking).
    optimizer.setRefinementParameters(lsqcpp::ArmijoBacktrackingParameter<float>{0.8, 1e-4, 1e-7, 10.0, 0});

    optimizer.setVerbosity(1);
    optimizer.setOutputStream(std::cerr);

    Eigen::VectorXf initialGuess(m_num_cells);
    for(Eigen::Index i = 0; i< initialGuess.size(); i++)
        initialGuess(i) = 10.0;

    auto result = optimizer.minimize(initialGuess);
    m_concentration = result.xval;
    RCLCPP_INFO(get_logger(), "Iterations: %ld", result.iterations); 
    RCLCPP_INFO(get_logger(), "Residual: %f", result.fval.norm()); 
}



void MapGenerator::getEnvironment()
{    
    // We are going to assume an empty map that's just the bounds of the measurements
    // for an example of how to actually do this for real, see the robot2023 branch
    std::ifstream file(m_input_filepath);
    if(!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Could not open file %s", m_input_filepath.c_str());
        raise(SIGTRAP);
    }

    uint num_cells_x, num_cells_y;
    //find environment dimensions (Bounding box)
    glm::vec2 min{FLT_MAX, FLT_MAX}, max{-FLT_MAX, -FLT_MAX};
    std::string line;
    while(std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform sensor = poseToTransform( mqtt_serialization::pose_from_json(json[m_sensor_name]).pose );
        tf2::Transform reflector = poseToTransform( mqtt_serialization::pose_from_json(json[m_reflector_name]).pose );

        min.x = std::min({(double)min.x, sensor.getOrigin().x(), reflector.getOrigin().x()});      
        min.y = std::min({(double)min.y, sensor.getOrigin().y(), reflector.getOrigin().y()});

        max.x = std::max({(double)max.x, sensor.getOrigin().x(), reflector.getOrigin().x()});      
        max.y = std::max({(double)max.y, sensor.getOrigin().y(), reflector.getOrigin().y()});       
    }

    RCLCPP_INFO(get_logger(), "Environment bounds: (%.2f, %.2f) --- (%.2f, %.2f)", min.x, min.y, max.x, max.y);
    num_cells_x = (max.x-min.x) / m_rayMarchResolution;
    num_cells_y = (max.y-min.y) / m_rayMarchResolution;

    file.close();
    
    m_num_cells = num_cells_x * num_cells_y;
    
    m_mapOrigin.x = min.x;
    m_mapOrigin.y = min.y;
    
    m_occupancy_map.resize(num_cells_x, std::vector<bool>(num_cells_y, true) );
}


void MapGenerator::runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, int ppmxm)
{
    constexpr float reflectorRadius = 0.26;

    static auto identity = [](const bool& b){return b;};
    
    auto doesNotCollideWithReflector = [reflectorPosition, reflectorRadius](const glm::vec2& position)
    {
        return glm::distance(position, reflectorPosition) > reflectorRadius;
    };

    DDA::_2D::RayMarchInfo rayData =  DDA::_2D::marchRay<bool>(origin, direction, 10, 
        {m_occupancy_map, m_mapOrigin, m_rayMarchResolution},
        identity, doesNotCollideWithReflector);
    
    for(const auto& [index, length] : rayData.lengthInCell)
    {
        uint columnIndex = index2Dto1D(index);
        m_lengthRayInCell(rowIndex,columnIndex) = length; 
        

#if RAYS_IMAGE       
        cv::Vec3b& r_image = rays_image.at<cv::Vec3b>(index.x, index.y); 
        
        double previous = 255-r_image[0];
        uint8_t value = (uint8_t) std::max(previous , 255 * (ppmxm/100.0));
        r_image = cv::Vec3b( 255-value , 255-value, 255 );
        
        //r_image += cv::Vec3b(length*100, length * 100, length * 100);
#endif
    }
}

void MapGenerator::writeHeatmap()
{
    cv::Mat image(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC1, cv::Scalar(0,0,0));
    
    float max = 0;
    for(int i = 0; i<m_concentration.size();i++)
        if(m_concentration[i]>max)
            max = m_concentration[i];


    for(int i = 0; i<m_occupancy_map.size(); i++)
    {
        for(int j=0; j<m_occupancy_map[0].size(); j++)
        {
            float concentration = m_concentration[i + j*m_occupancy_map.size()];
            image.at<uint8_t>(i, j) = (concentration / max) * 255;
        }
    }

    cv::Mat img_color;
    cv::applyColorMap(image, img_color, cv::COLORMAP_JET);

    //occupancy
    {
        for(int i = 0; i<m_occupancy_map.size(); i++)
        {
            for(int j=0; j<m_occupancy_map[0].size(); j++)
            {
                if(!m_occupancy_map[i][j])
                    img_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0 , 0);
            }
        }
    }

    std::filesystem::path inputPath(m_input_filepath);
    std::string outputPath = fmt::format("concentration_map.png");
    cv::imwrite(outputPath, img_color);


    RCLCPP_INFO(get_logger(), "MAX CONCENTRATION VALUE: %.3f ppm", max);
    RCLCPP_INFO(get_logger(), "MAP WAS GENERATED AT PATH: %s", outputPath.c_str());
}



int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MapGenerator>();
    
    node->getEnvironment();
    node->readFile();
    node->solve();
    node->writeHeatmap();

    rclcpp::shutdown();
    return 0;
}