#include <fmt/format.h>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/logging.hpp>
#include <tdlas_mapping/MapGenerator.hpp>

MapGenerator::MapGenerator(const MapGeneratorOptions& options, std::vector<double>& concentrations)
    : m_options(options), m_concentration(concentrations)
{
    m_concentration.resize(options.numCells, options.prior);
    m_concentrationPrior.resize(options.numCells);
}

void MapGenerator::SetMap(const std::vector<std::vector<bool>>& occupancy_map, glm::vec2 mapOrigin, float rayMarchResolution)
{
    m_occupancy_map = occupancy_map;
    m_rayMarchResolution = rayMarchResolution;
    m_mapOrigin = mapOrigin;

    m_raysImage.create(cv::Size(m_occupancy_map[0].size(), m_occupancy_map.size()), CV_8UC3);
    m_raysImage.setTo(cv::Vec3b(255, 255, 255));

    for (int i = 0; i < m_occupancy_map.size(); i++)
    {
        for (int j = 0; j < m_occupancy_map[0].size(); j++)
        {
            if (!m_occupancy_map[i][j])
                m_raysImage.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
        }
    }
}

void MapGenerator::SetMeasurements(const std::vector<TDLASMeasurement>& measurements)
{
    for (int i = 0; i < measurements.size(); i++)
    {
        auto& m = measurements[i];
        m_measurements.push_back(m.ppmxm);
        m_lengthRayInCell.push_back(std::vector<double>(m_concentration.size(), 0.0));

        runDDA(m.origin, m.direction, m.reflectorPosition, i, m.ppmxm);
    }
    cv::imwrite("rays.png", m_raysImage);
}

void MapGenerator::runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, float ppmxm)
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

        if (m_options.useRayPrior)
            m_concentration[columnIndex] = m_concentrationPrior[columnIndex].Update(ppmxm / rayData.totalLength);

#define DEBUG_POSITIONS 0
#if DEBUG_POSITIONS
        // Draw only the final positions
        glm::ivec2 indices = (reflectorPosition - m_mapOrigin) / m_rayMarchResolution;
        cv::Vec3b& r_image = rays_image.at<cv::Vec3b>(indices.x, indices.y);
        r_image = cv::Vec3b(0, 0, 255);

        indices = (origin - m_mapOrigin) / m_rayMarchResolution;
        rays_image.at<cv::Vec3b>(indices.x, indices.y) = cv::Vec3b(0, 255, 0);
#else
        // Draw the rays
        cv::Vec3b& r_image = m_raysImage.at<cv::Vec3b>(index.x, index.y);
        double previous = 255 - r_image[0];
        double this_value = 255 * std::min(1.0, ppmxm / 150.0);

        uint8_t value = (uint8_t)std::max(previous, this_value);
        if (r_image != cv::Vec3b(0, 255, 0))
            r_image = cv::Vec3b(255 - value, 255 - value, 255);
#endif
    }
}

std::vector<double> MapGenerator::Solve(const std::vector<ceres::IterationCallback*>& callbacks)
{
    google::InitGoogleLogging("Ceres");
    ceres::Problem problem;

    RayFunctor::lengthInCell = m_lengthRayInCell;
    RayFunctor::lambda = m_options.lambda;
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
    LambdaFunctor::lambda = m_options.lambda;
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
    options.minimizer_type = ceres::LINE_SEARCH; // for some reason, using TRUST_REGION does not work. The solver reports a huge negative cost_change
                                                 // every iteration and the answer remains untouched
    options.trust_region_strategy_type = ceres::LEVENBERG_MARQUARDT;
    options.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    options.use_explicit_schur_complement = true;

    options.num_threads = 10;
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;

    options.max_num_iterations = 10000;
    options.function_tolerance = 1e-12;
    options.gradient_tolerance = 1e-15;

    options.update_state_every_iteration = true;
    for (ceres::IterationCallback* c : callbacks) options.callbacks.push_back(c);

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    RCLCPP_INFO(rclcpp::get_logger("MapGenerator"), "Done! Took %.2fs", summary.total_time_in_seconds);
    RCLCPP_INFO(rclcpp::get_logger("MapGenerator"), "Summary:\n%s", summary.FullReport().c_str());

    return m_concentration;
}

void MapGenerator::WriteHeatmap()
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

    std::string outputPath = fmt::format("concentration_map.png");
    cv::imwrite(outputPath, img_color);

    RCLCPP_INFO(rclcpp::get_logger("MapGenerator"), "MAX CONCENTRATION VALUE: %.3f ppm", max);
    RCLCPP_DEBUG(rclcpp::get_logger("MapGenerator"), "MAP WAS GENERATED AT PATH: %s", outputPath.c_str());
}
