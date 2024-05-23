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

#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>

#define RAYS_IMAGE 1

#if RAYS_IMAGE
static cv::Mat rays_image;
#endif

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TDLAS = olfaction_msgs::msg::TDLAS;
using namespace std::chrono_literals;

static TDLAS jsonToTDLAS(const nlohmann::json& json)
{
    TDLAS tdlas;
    tdlas.average_ppmxm = (double)json["average_ppmxm"].get<int>();
    tdlas.average_absorption_strength = json["average_absorption_strength"].get<double>();
    tdlas.average_reflection_strength = json["average_reflection_strength"].get<double>();
    return tdlas;
}

MapGenerator::MapGenerator() : Node("MapGenerator")
{
    m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.05f);
    m_lambda = declare_parameter<float>("lambda", 0.01);
    m_prior = declare_parameter<double>("prior", 0.0);
    m_input_filepath = declare_parameter<std::string>("filepath", "measurement_log");
    m_sensor_name = declare_parameter<std::string>("sensor_name", "sensorTF");
    m_reflector_name = declare_parameter<std::string>("reflector_name", "reflectorTF");
}

void MapGenerator::getEnvironment()
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
        tf2::Transform sensor = poseToTransform(mqtt_serialization::pose_from_json(json[m_sensor_name]).pose);
        tf2::Transform reflector = poseToTransform(mqtt_serialization::pose_from_json(json[m_reflector_name]).pose);

        min.x = std::min({(double)min.x, sensor.getOrigin().x(), reflector.getOrigin().x()});
        min.y = std::min({(double)min.y, sensor.getOrigin().y(), reflector.getOrigin().y()});

        max.x = std::max({(double)max.x, sensor.getOrigin().x(), reflector.getOrigin().x()});
        max.y = std::max({(double)max.y, sensor.getOrigin().y(), reflector.getOrigin().y()});
    }

    // a little bit of arbitrary padding around the used area
    max += glm::vec2{m_rayMarchResolution * 4, m_rayMarchResolution * 4};
    min += glm::vec2{-m_rayMarchResolution * 4, -m_rayMarchResolution * 4};

    RCLCPP_INFO(get_logger(), "Environment bounds: (%.2f, %.2f) --- (%.2f, %.2f)", min.x, min.y, max.x, max.y);
    num_cells_x = std::ceil((max.x - min.x) / m_rayMarchResolution);
    num_cells_y = std::ceil((max.y - min.y) / m_rayMarchResolution);

    file.close();

    m_num_cells = num_cells_x * num_cells_y;

    m_mapOrigin.x = min.x;
    m_mapOrigin.y = min.y;

    m_occupancy_map.resize(num_cells_x, std::vector<bool>(num_cells_y, true));
}

void MapGenerator::readFile()
{
    std::ifstream file(m_input_filepath);

    if (!file.is_open())
    {
        RCLCPP_ERROR(get_logger(), "Could not open file %s", m_input_filepath.c_str());
        raise(SIGTRAP);
    }

    // Count the number of entries and size the matrices accordingly
    int numberOfMeasurements = 0;
    std::string line;
    while (std::getline(file, line))
        numberOfMeasurements++;

    m_measurements.resize(numberOfMeasurements);
    m_lengthRayInCell.resize(numberOfMeasurements, std::vector<double>(m_num_cells, m_prior));
    m_concentration.resize(m_num_cells);

    // restart the ifstream
    file.clear();
    file.seekg(0, std::ios::beg);

    int measurementIndex = 0;

#if RAYS_IMAGE
    rays_image.create(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC3);
    rays_image.setTo(cv::Vec3b(255, 255, 255));
#endif

    uint numLines = 0;
    while (std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform sensor = poseToTransform(mqtt_serialization::pose_from_json(json[m_sensor_name]).pose);
        tf2::Transform reflector = poseToTransform(mqtt_serialization::pose_from_json(json[m_reflector_name]).pose);
        TDLAS tdlas = jsonToTDLAS(json["reading"]);

        m_measurements[measurementIndex] = tdlas.average_ppmxm;

        // fill in the cell raytracing thing
        glm::vec2 rayOrigin = glm::fromTF(sensor.getOrigin());
        glm::vec2 reflectorPosition = glm::fromTF(reflector.getOrigin());
        // glm::vec2 rayDirection = glm::fromTF( tf2::quatRotate(sensor.getRotation(), {0,1,0}) );
        glm::vec2 rayDirection = glm::normalize(reflectorPosition - rayOrigin);

        runDDA(rayOrigin, rayDirection, reflectorPosition, measurementIndex, tdlas.average_ppmxm);
        measurementIndex++;
        numLines++;

#if RAYS_IMAGE
        glm::vec2 indices = (rayOrigin - m_mapOrigin) / m_rayMarchResolution;
        rays_image.at<cv::Vec3b>(indices.x, indices.y) = cv::Vec3b(0, 255, 0);
#endif
    }
    RCLCPP_INFO(get_logger(), "Number of lines parsed: %u", numLines);
    file.close();

#if RAYS_IMAGE
    for (int i = 0; i < m_occupancy_map.size(); i++)
    {
        for (int j = 0; j < m_occupancy_map[0].size(); j++)
        {
            if (!m_occupancy_map[i][j])
                rays_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
        }
    }

    cv::imwrite("rays.png", rays_image);
#endif
}

void MapGenerator::runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, int ppmxm)
{
    constexpr float reflectorRadius = 0.26;

    static auto identity = [](const bool& b) { return b; };

    auto doesNotCollideWithReflector = [reflectorPosition, reflectorRadius](const glm::vec2& position) {
        return glm::distance(position, reflectorPosition) > reflectorRadius;
    };

    DDA::_2D::RayMarchInfo rayData = DDA::_2D::marchRay<bool>(origin, direction, FLT_MAX, {m_occupancy_map, m_mapOrigin, m_rayMarchResolution},
                                                              identity, doesNotCollideWithReflector);

    for (const auto& [index, length] : rayData.lengthInCell)
    {
        uint columnIndex = index2Dto1D(index);
        m_lengthRayInCell[rowIndex][columnIndex] = length;

#if RAYS_IMAGE
#define DEBUG_POSITIONS 0
#if DEBUG_POSITIONS
        // DEBUG - Draw only the final positions
        glm::ivec2 indices = (reflectorPosition - m_mapOrigin) / m_rayMarchResolution;
        cv::Vec3b& r_image = rays_image.at<cv::Vec3b>(indices.x, indices.y);
        r_image = cv::Vec3b(0, 0, 255);

        indices = (origin - m_mapOrigin) / m_rayMarchResolution;
        rays_image.at<cv::Vec3b>(indices.x, indices.y) = cv::Vec3b(0, 255, 0);
#else
        // Draw the rays
        cv::Vec3b& r_image = rays_image.at<cv::Vec3b>(index.x, index.y);
        double previous = 255 - r_image[0];
        double this_value = 255 * std::min(1.0, ppmxm / 150.0);

        uint8_t value = (uint8_t)std::max(previous, this_value);
        if (r_image != cv::Vec3b(0, 255, 0))
            r_image = cv::Vec3b(255 - value, 255 - value, 255);
#endif
#endif
    }
}

struct CostFunctor
{
    static inline std::vector<std::vector<double>> lengthInCell;
    static inline std::vector<double> measurements;
    static inline double lambda;

    int rowIndex; //each CostFunctor object works on a single row of the matrix
    CostFunctor(int i) : rowIndex(i)
    {}

    template <typename T> 
    bool operator()(T const* const* x, T* residuals) const
    {
        const T* concentrations = *x;
        T predictedReading = DotProduct(lengthInCell[rowIndex], concentrations);
        T concentrationsNorm = norm(concentrations, lengthInCell[rowIndex].size());
        residuals[0] = ceres::abs(predictedReading - measurements[rowIndex]) + concentrationsNorm * concentrationsNorm * lambda;
        return true;
    }

    template <typename T> 
    T DotProduct(const std::vector<double>& row, const T* concentrations) const
    {
        T sum{0};
        for (int i = 0; i < row.size(); i++)
            sum += row[i] * concentrations[i];
        return sum;
    }

    template <typename T>
    T norm(const T* array, size_t size) const
    {
        T sum{0};
        for(int i = 0; i < size; i++)
        {
            sum += array[i] * array[i];
        }
        return sum;//ceres::sqrt(sum+1e-8);
    }
};

void MapGenerator::solve()
{
    google::InitGoogleLogging("Ceres");
    ceres::Problem problem;

    CostFunctor::lengthInCell = m_lengthRayInCell;
    CostFunctor::lambda = m_lambda;
    CostFunctor::measurements = m_measurements;

    for(int i = 0; i<m_lengthRayInCell.size(); i++)
    {
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<CostFunctor, 4>(new CostFunctor(i));
        cost_function->SetNumResiduals(1);
        cost_function->AddParameterBlock(m_concentration.size());
        problem.AddResidualBlock(cost_function, nullptr, m_concentration.data());
    }
    
    for(int i = 0; i<m_concentration.size(); i++)
    {
        problem.SetParameterLowerBound(m_concentration.data(), i, 0.0);
        problem.SetParameterUpperBound(m_concentration.data(), i, 1000.0);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.minimizer_progress_to_stdout = false;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;
    options.max_num_iterations=100;
    options.num_threads = 10;


    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    RCLCPP_INFO(get_logger(), "Done! Took %.2fs", summary.total_time_in_seconds);
    RCLCPP_INFO(get_logger(), "Summary:\n%s", summary.BriefReport().c_str());
}

void MapGenerator::writeHeatmap()
{
    cv::Mat image(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC1, cv::Scalar(0, 0, 0));

    float max = 0;
    for (int i = 0; i < m_concentration.size(); i++)
        if (m_concentration[i] > max)
            max = m_concentration[i];

    for (int i = 0; i < m_occupancy_map.size(); i++)
    {
        for (int j = 0; j < m_occupancy_map[0].size(); j++)
        {
            float concentration = m_concentration[i + j * m_occupancy_map.size()];
            image.at<uint8_t>(i, j) = (uint8_t)((concentration / max) * 255);
        }
    }

    cv::Mat img_color;
    cv::applyColorMap(image, img_color, cv::COLORMAP_JET);

    // occupancy
    {
        for (int i = 0; i < m_occupancy_map.size(); i++)
        {
            for (int j = 0; j < m_occupancy_map[0].size(); j++)
            {
                if (!m_occupancy_map[i][j])
                    img_color.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
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