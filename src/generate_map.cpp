#include <tdlas_mapping/generate_map.h>
#include <fstream>
#include <json/PoseJSON.hpp>
#include <tdlas_mapping/common.h>
#include <json/Utils.hpp>
#include <tf2/LinearMath/Transform.h>

#include <opencv4/opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/imgcodecs.hpp>
#include <fmt/core.h>
#include <filesystem>

#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>

#include <ament_imgui/ament_imgui.h>

static cv::Mat rays_image;

using PoseStamped = geometry_msgs::msg::PoseStamped;
using TDLAS = olfaction_msgs::msg::TDLAS;
using namespace std::chrono_literals;

MapGenerator::MapGenerator() : Node("MapGenerator")
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
    m_lengthRayInCell.resize(numberOfMeasurements, std::vector<double>(m_num_cells, 0.0));
    m_concentration.resize(m_num_cells, m_prior);
    m_concentrationPrior.resize(m_num_cells);

    // restart the ifstream
    file.clear();
    file.seekg(0, std::ios::beg);

    int measurementIndex = 0;

    rays_image.create(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC3);
    rays_image.setTo(cv::Vec3b(255, 255, 255));

    uint numLines = 0;
    while (std::getline(file, line))
    {
        auto json = nlohmann::json::parse(line);
        tf2::Transform sensor = poseToTransform(jsonSerialization::pose_from_json(json[m_sensor_name]).pose);
        tf2::Transform reflector = poseToTransform(jsonSerialization::pose_from_json(json[m_reflector_name]).pose);
        TDLAS tdlas = jsonSerialization::Utils::jsonToTDLAS(json["reading"]);

        m_measurements[measurementIndex] = tdlas.average_ppmxm;

        // fill in the cell raytracing thing
        glm::vec2 rayOrigin = glm::fromTF(sensor.getOrigin());
        glm::vec2 reflectorPosition = glm::fromTF(reflector.getOrigin());
        // glm::vec2 rayDirection = glm::fromTF( tf2::quatRotate(sensor.getRotation(), {0,1,0}) );
        glm::vec2 rayDirection = glm::normalize(reflectorPosition - rayOrigin);

        runDDA(rayOrigin, rayDirection, reflectorPosition, measurementIndex, tdlas.average_ppmxm);
        measurementIndex++;
        numLines++;

        glm::vec2 indices = (rayOrigin - m_mapOrigin) / m_rayMarchResolution;
        rays_image.at<cv::Vec3b>(indices.x, indices.y) = cv::Vec3b(0, 255, 0);
    }
    RCLCPP_INFO(get_logger(), "Number of lines parsed: %u", numLines);
    file.close();

    for (int i = 0; i < m_occupancy_map.size(); i++)
    {
        for (int j = 0; j < m_occupancy_map[0].size(); j++)
        {
            if (!m_occupancy_map[i][j])
                rays_image.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
        }
    }

    cv::imwrite("rays.png", rays_image);
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

        if (m_useRayPrior)
            m_concentration[columnIndex] = m_concentrationPrior[columnIndex].Update( 100 *ppmxm / rayData.totalLength);

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
    }
}

struct RayFunctor
{
    static inline std::vector<std::vector<double>> lengthInCell;
    static inline std::vector<double> measurements;
    static inline double lambda;

    int rowIndex;             // each CostFunctor object works on a single row of the matrix
    std::vector<int> indices; // each instance of the functor uses only a subset of the cells as parameters. this tells you which ones
    RayFunctor(int i) : rowIndex(i)
    {}

    template <typename T>
    bool operator()(T const* const* x, T* residuals) const
    {
        const T* concentrations = *x;
        T predictedReading = DotProduct(lengthInCell[rowIndex], concentrations);
        residuals[0] = ceres::abs(predictedReading - measurements[rowIndex]);
        return true;
    }

    template <typename T>
    T DotProduct(const std::vector<double>& row, const T* concentrations) const
    {
        T sum{0};
        for (int i = 0; i < indices.size(); i++)
            sum += row[indices[i]] * concentrations[i];
        return sum;
    }
};

struct LambdaFunctor
{
    static inline double lambda;
    int size;
    LambdaFunctor(int _size) : size(_size)
    {}

    template <typename T>
    bool operator()(T const* const* x, T* residuals) const
    {
        const T* concentrations = *x;
        for (int i = 0; i < size; i++)
            if (concentrations[i] < T{0})
                return false;

        T concentrationsNorm = squaredNorm(concentrations, size);
        residuals[0] = concentrationsNorm * lambda;
        return true;
    }

    template <typename T>
    T squaredNorm(const T* array, size_t size) const
    {
        T sum{0};
        for (int i = 0; i < size; i++)
        {
            sum += array[i] * array[i];
            // sum += ceres::abs(array[i] -T{i}); //test
        }
        // return ceres::sqrt(sum+1e-8);
        return sum;
    }
};

class IterationCallback : public ceres::IterationCallback
{
public:
    explicit IterationCallback(MapGenerator& _mapGenerator) : mapGenerator(_mapGenerator)
    {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary)
    {
        mapGenerator.writeHeatmap();
        mapGenerator.publishMarkers();
        if (rclcpp::ok())
            return ceres::SOLVER_CONTINUE;
        else
            return ceres::SOLVER_ABORT;
    }

private:
    MapGenerator& mapGenerator;
};

void MapGenerator::solve()
{
    google::InitGoogleLogging("Ceres");
    ceres::Problem problem;

    RayFunctor::lengthInCell = m_lengthRayInCell;
    RayFunctor::lambda = m_lambda;
    RayFunctor::measurements = m_measurements;

    for (int i = 0; i < m_lengthRayInCell.size(); i++)
    {
        auto* functor = new RayFunctor(i);
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<RayFunctor, 4>(functor);
        std::vector<double*> params;
        for (int j = 0; j < m_lengthRayInCell[i].size(); j++)
            if (m_lengthRayInCell[i][j] != 0)
            {
                functor->indices.push_back(j);
                params.push_back(&m_concentration[j]);
                cost_function->AddParameterBlock(1);
            }
        cost_function->SetNumResiduals(1);
        problem.AddResidualBlock(cost_function, nullptr, params);

        // for (int j = 0; j < params.size(); j++)
        //     problem.SetParameterLowerBound(params[j], 0, 0.0);
    }

    // separate functor to apply the lambda to the whole map
    LambdaFunctor::lambda = m_lambda;
    {
        auto* cost_function = new ceres::DynamicAutoDiffCostFunction<LambdaFunctor, 4>(new LambdaFunctor(m_concentration.size()));
        std::vector<double*> params;

        for (int j = 0; j < m_concentration.size(); j++)
        {
            cost_function->AddParameterBlock(1);
            params.push_back(&m_concentration[j]);
        }

        cost_function->SetNumResiduals(1);
        problem.AddResidualBlock(cost_function, nullptr, params);
    }

    ceres::Solver::Options options;
    options.minimizer_type = ceres::LINE_SEARCH; //for some reason, using TRUST_REGION does not work. The solver reports a huge negative cost_change every iteration and the answer remains untouched
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_explicit_schur_complement = true;

    options.num_threads = 10;
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;

    options.max_num_iterations = 10000;
    options.function_tolerance = 1e-9;
    options.gradient_tolerance = 1e-12;

    options.update_state_every_iteration = true;
    options.callbacks.push_back(new IterationCallback(*this));

    ceres::Solver::Summary summary;
    Solve(options, &problem, &summary);
    
    //Attempt to perturb the solution slightly to escape local minima. Not really working at all, stops immediately and returns the noisy input as a valid solution
    //for(int i = 0; i<3; i++)
    //{
    //    RCLCPP_WARN(get_logger(), "ANNEALING ITERATION %d", i);
    //    double max = *std::max_element(m_concentration.begin(), m_concentration.end());
    //    addNoiseToSolution(0.05f * max);
    //    Solve(options, &problem, &summary);
    //}


    RCLCPP_INFO(get_logger(), "Done! Took %.2fs", summary.total_time_in_seconds);
    RCLCPP_INFO(get_logger(), "Summary:\n%s", summary.FullReport().c_str());
}

void MapGenerator::addNoiseToSolution(float stdev)
{
    for(int i = 0; i< m_concentration.size(); i++)
    {
        m_concentration[i] += RandomFromGaussian(0, stdev);
        m_concentration[i] = std::max(m_concentration[i], 0.0);
    }
}


void MapGenerator::writeHeatmap()
{
    cv::Mat image(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC1, cv::Scalar(0, 0, 0));

    float max = -FLT_MAX;
    for (int i = 0; i < m_concentration.size(); i++)
        if (m_concentration[i] > max)
            max = m_concentration[i];

    for (int i = 0; i < m_occupancy_map.size(); i++)
    {
        for (int j = 0; j < m_occupancy_map[0].size(); j++)
        {
            float concentration = m_concentration[i + j * m_occupancy_map.size()];
            if (max == 0)
                image.at<uint8_t>(i, j) = 0;
            else
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
    RCLCPP_DEBUG(get_logger(), "MAP WAS GENERATED AT PATH: %s", outputPath.c_str());
}

void MapGenerator::publishMarkers()
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

void MapGenerator::renderGUI()
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
    auto node = std::make_shared<MapGenerator>();

    std::thread renderThread([node]() { node->renderGUI(); });

    node->getEnvironment();
    node->readFile();
    node->solve();
    node->writeHeatmap();

    rclcpp::Rate rate(30);
    while (rclcpp::ok())
    {
        node->publishMarkers();
    }

    renderThread.join();
    rclcpp::shutdown();
    return 0;
}