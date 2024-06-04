#include <ament_imgui/ament_imgui.h>
#include <tdlas_mapping/Logging.hpp>
#include <tdlas_mapping/MapGenerator.hpp>
#include <visualization_msgs/msg/marker.hpp>
#define GLM_ENABLE_EXPERIMENTAL
#include <glm/gtx/rotate_vector.hpp>

#include <filesystem>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

using visualization_msgs::msg::Marker;

static constexpr float degToRad = M_PI / 180.0;

class Simulation;

class IterationCallback : public ceres::IterationCallback
{
public:
    explicit IterationCallback(Simulation& _node, MapGenerator& _mapGenerator) : node(_node), mapGenerator(_mapGenerator)
    {}

    ceres::CallbackReturnType operator()(const ceres::IterationSummary& summary);

private:
    Simulation& node;
    MapGenerator& mapGenerator;
};

class Simulation : public rclcpp::Node
{
public:
    Simulation() : Node("MapSimulation")
    {
        m_rayMarchResolution = declare_parameter<float>("rayMarchResolution", 0.05f);
        m_lambda = declare_parameter<float>("lambda", 0.01);
        m_prior = declare_parameter<double>("prior", 0.0);
        m_useRayPrior = declare_parameter<bool>("useRayPrior", false);
        m_gridMarkerPub = create_publisher<Marker>("/concentration_markers", 1);
        m_sourceMarkerPub = create_publisher<Marker>("/ground_truth", 1);
        m_markerColorLimits = glm::vec2(0, 10);

        m_mapSize.x = declare_parameter<float>("mapSizeX", 10);
        m_mapSize.y = declare_parameter<float>("mapSizeY", 10);

        m_sourcePos.x = declare_parameter<float>("sourcePosX", 5);
        m_sourcePos.y = declare_parameter<float>("sourcePosY", 5);

        m_sensorNoise = declare_parameter<double>("sensorNoise", 10);
        m_numMeasurementsPerPosition = declare_parameter<int>("numMeasurementsPerPosition", 1);
        declare_parameter<std::string>("GTConcentrationImage", "");
    }

    void run()
    {
        createMap();
        createMeasurements();

        std::thread renderThread([this]() { renderGUI(); });
        if (declare_parameter<bool>("solve", true))
            solve();

        rclcpp::Rate rate(30);
        while (rclcpp::ok())
        {
            publishMarkers();
            rate.sleep();
        }
        renderThread.join();
    }

    void publishMarkers()
    {
        Marker gridMarker;
        gridMarker.header.frame_id = "map";
        gridMarker.header.stamp = now();
        gridMarker.type = Marker::POINTS;
        gridMarker.scale.x = gridMarker.scale.y = m_rayMarchResolution;

        for (int i = 0; i < m_occupancy.size(); i++)
        {
            for (int j = 0; j < m_occupancy[0].size(); j++)
            {
                if (m_occupancy[i][j])
                {
                    gridMarker.points.push_back(glm::toPoint(glm::vec2(i, j) * m_rayMarchResolution));

                    float ppmxm = m_concentration[i + j * m_occupancy.size()];
                    std_msgs::msg::ColorRGBA color = valueToColor(ppmxm, m_markerColorLimits.x, m_markerColorLimits.y, valueColorMode::Linear);
                    color.a = m_markerAlpha;
                    gridMarker.colors.push_back(color);
                }
            }
        }

        m_gridMarkerPub->publish(gridMarker);

        Marker sourceMarker;
        sourceMarker.header.frame_id = "map";
        sourceMarker.header.stamp = now();
        sourceMarker.type = Marker::POINTS;
        sourceMarker.points.push_back(glm::toPoint(m_sourcePos));
        sourceMarker.colors.push_back(makeColor(1, 0, 1, 1));
        sourceMarker.scale.x = sourceMarker.scale.y = m_rayMarchResolution;
        m_sourceMarkerPub->publish(sourceMarker);
    }

private:
    std::optional<MapGenerator> mapGenerator;
    std::vector<double> m_concentration;
    std::vector<std::vector<bool>> m_occupancy;
    glm::vec2 m_mapSize;
    glm::vec2 m_sourcePos;
    float m_lambda;
    bool m_useRayPrior;
    double m_prior;
    float m_rayMarchResolution;
    int m_numMeasurementsPerPosition;
    double m_sensorNoise;

    rclcpp::Publisher<Marker>::SharedPtr m_gridMarkerPub;
    rclcpp::Publisher<Marker>::SharedPtr m_sourceMarkerPub;
    glm::vec2 m_markerColorLimits;
    float m_markerAlpha = 0.5;

    void createMap()
    {
        int numCellsX = m_mapSize.x / m_rayMarchResolution;
        int numCellsY = m_mapSize.y / m_rayMarchResolution;

        m_occupancy.resize(numCellsX, std::vector<bool>(numCellsY, true));

        MapGeneratorOptions options;
        options.lambda = m_lambda;
        options.numCells = numCellsX * numCellsY;
        options.prior = m_prior;
        options.useRayPrior = m_useRayPrior;
        mapGenerator.emplace(options, m_concentration);
        mapGenerator->SetMap(m_occupancy, {0.0, 0.0}, m_rayMarchResolution);
    }

    void createMeasurements()
    {
        // std::vector<TDLASMeasurement> measurements = verticalAndHorizontalSweep();
        // std::vector<TDLASMeasurement> measurements = twoCorners();
        // std::vector<TDLASMeasurement> measurements = threeCorners();
        // std::vector<TDLASMeasurement> measurements = fourCorners();
        // std::vector<TDLASMeasurement> measurements = axialSinglePoint();
        // std::vector<TDLASMeasurement> measurements = axialPointPlane();
        //std::vector<TDLASMeasurement> measurements = axialPointArc();
        std::vector<TDLASMeasurement> measurements = axialPlane();
        mapGenerator->SetMeasurements(measurements);
    }

    void solve()
    {
        mapGenerator->Solve({new IterationCallback(*this, *mapGenerator)});
    }

    void renderGUI()
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

    double simulateMeasurement(glm::vec2 origin, glm::vec2 direction, glm::vec2 reflectorPosition)
    {
        constexpr float reflectorRadius = 0.26;

        static auto identity = [](const bool& b) { return b; };

        auto doesNotCollideWithReflector = [reflectorPosition, reflectorRadius](const glm::vec2& position) {
            return glm::distance(position, reflectorPosition) > reflectorRadius;
        };

        DDA::_2D::RayMarchInfo rayData =
            DDA::_2D::marchRay<bool>(origin, direction, FLT_MAX, {m_occupancy, {0, 0}, m_rayMarchResolution}, identity, doesNotCollideWithReflector);

        double sum = 0;
        for (const auto& [index, length] : rayData.lengthInCell)
        {
            sum += concentrationAt(glm::vec2(index) * m_rayMarchResolution) * length;
        }

        return std::max(0.0, sum + RandomFromGaussian(0, m_sensorNoise));
    }

    double concentrationAt(glm::vec2 coords)
    {
        glm::ivec2 indices = coords / m_rayMarchResolution;
        static const std::string path = get_parameter_or<std::string>("GTConcentrationImage", "");
        if (std::filesystem::exists(path))
        {
            static cv::Mat GTConcentration = cv::imread(path, cv::IMREAD_GRAYSCALE);
            indices.x = std::clamp(indices.x, 0, GTConcentration.size().width);
            indices.y = std::clamp(indices.y, 0, GTConcentration.size().height);
            uint8_t v = GTConcentration.at<uint8_t>(indices.x, indices.y);
            return v;
        }
        else
        {
            if (indices == glm::ivec2(m_sourcePos / m_rayMarchResolution))
                return 700;
            else
                return 1.9;
        }
    }

    //----------------------------------------------------------
    // PATTERNS
    //----------------------------------------------------------
    std::vector<TDLASMeasurement> verticalAndHorizontalSweep()
    {
        std::vector<TDLASMeasurement> measurements;

        // vertical
        for (int i = 0; i < m_occupancy.size(); i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(i, 0) * m_rayMarchResolution;
                m.direction = {0, 1};
                m.reflectorPosition = glm::vec2(i, m_occupancy.size()) * m_rayMarchResolution;
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // horizontal
        for (int i = 0; i < m_occupancy[0].size(); i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0, i) * m_rayMarchResolution;
                m.direction = {1, 0};
                m.reflectorPosition = glm::vec2(m_occupancy[0].size(), i) * m_rayMarchResolution;
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> twoCorners()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float numSamples = 100;
        glm::vec2 direction(20, 0);

        // first
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.1 * m_mapSize.x, 0.1 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // second
        direction = {0, 20};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.9 * m_mapSize.x, 0.1 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> threeCorners()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float numSamples = 150;
        glm::vec2 direction(20, 0);

        // first
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.1 * m_mapSize.x, 0.1 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // second
        direction = {0, 20};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.9 * m_mapSize.x, 0.1 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // third
        direction = {0, -20};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.1 * m_mapSize.x, 0.9 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> fourCorners()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float numSamples = 100;
        glm::vec2 direction(20, 0);

        // first
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.1 * m_mapSize.x, 0.1 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // second
        direction = {0, 20};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.9 * m_mapSize.x, 0.1 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // third
        direction = {0, -20};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.1 * m_mapSize.x, 0.9 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }

        // fourth
        direction = {-20, -0};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                m.origin = glm::vec2(0.9 * m_mapSize.x, 0.9 * m_mapSize.y);
                m.reflectorPosition = m.origin + glm::rotate(direction, (float)M_PI / 2 * i / numSamples);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> axialSinglePoint()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float numSamples = 100;
        glm::vec2 direction(9, 0);

        direction = {-9, -0};
        for (int i = 0; i < numSamples; i++)
        {
            for (int j = 0; j < m_numMeasurementsPerPosition; j++)
            {
                TDLASMeasurement m;
                glm::vec2 rotatedDir = glm::rotate(direction, (float)M_PI * 2 * i / numSamples);
                m.origin = m_mapSize * 0.5f + rotatedDir;
                m.reflectorPosition = m_mapSize * 0.5f - rotatedDir;
                m.direction = glm::normalize(m.reflectorPosition - m.origin);
                m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                measurements.push_back(m);
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> axialPointPlane()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float planeHalfExtents = 7;
        constexpr float planeStepSize = 0.2;

        constexpr float numSamples = 8;
        glm::vec2 direction(7, 0);

        for (int i = 0; i < numSamples; i++)
        {
            TDLASMeasurement m;
            glm::vec2 rotatedDir = glm::rotate(direction, (float)M_PI * 2 * i / numSamples);
            glm::vec2 offsetDir = glm::normalize(glm::rotate(rotatedDir, (float)M_PI * 0.5f));
            m.origin = m_mapSize * 0.5f + rotatedDir;

            for (float offsetAmount = -planeHalfExtents; offsetAmount < planeHalfExtents; offsetAmount += planeStepSize)
            {
                m.reflectorPosition = m_mapSize * 0.5f - rotatedDir + offsetDir * offsetAmount;
                m.direction = glm::normalize(m.reflectorPosition - m.origin);

                for (int j = 0; j < m_numMeasurementsPerPosition; j++)
                {
                    m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                    measurements.push_back(m);
                }
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> axialPointArc()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float arcHalfLength = 90.0f * degToRad;
        constexpr float arcStep = 0.5 * degToRad;

        constexpr float numSamples = 8;
        glm::vec2 direction(7, 0);

        for (int i = 0; i < numSamples; i++)
        {
            TDLASMeasurement m;
            glm::vec2 rotatedDir = glm::rotate(direction, (float)M_PI * 2 * i / numSamples);
            m.origin = m_mapSize * 0.5f + rotatedDir;

            for (float offsetAmount = -arcHalfLength; offsetAmount < arcHalfLength; offsetAmount += arcStep)
            {
                m.reflectorPosition = m.origin + glm::rotate(-2.0f * rotatedDir, offsetAmount);
                m.direction = glm::normalize(m.reflectorPosition - m.origin);

                for (int j = 0; j < m_numMeasurementsPerPosition; j++)
                {
                    m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                    measurements.push_back(m);
                }
            }
        }
        return measurements;
    }

    std::vector<TDLASMeasurement> axialPlane()
    {
        std::vector<TDLASMeasurement> measurements;

        constexpr float planeHalfExtents = 5;
        constexpr float planeStepSize = 0.1;

        constexpr float numSamples = 150;
        glm::vec2 direction(9, 0);

        for (int i = 0; i < numSamples; i++)
        {
            TDLASMeasurement m;
            glm::vec2 rotatedDir = glm::rotate(direction, (float)M_PI * 2 * i / numSamples);
            glm::vec2 offsetDir = glm::normalize(glm::rotate(rotatedDir, (float)M_PI * 0.5f));

            for (float offsetAmount = -planeHalfExtents; offsetAmount < planeHalfExtents; offsetAmount += planeStepSize)
            {
                m.origin = m_mapSize * 0.5f + rotatedDir + offsetDir * offsetAmount;
                m.reflectorPosition = m_mapSize * 0.5f - rotatedDir + offsetDir * offsetAmount;
                m.direction = glm::normalize(m.reflectorPosition - m.origin);

                for (int j = 0; j < m_numMeasurementsPerPosition; j++)
                {
                    m.ppmxm = simulateMeasurement(m.origin, m.direction, m.reflectorPosition);
                    measurements.push_back(m);
                }
            }
        }
        return measurements;
    }
};

ceres::CallbackReturnType IterationCallback::operator()(const ceres::IterationSummary& summary)
{
    mapGenerator.WriteHeatmap();
    node.publishMarkers();
    if (rclcpp::ok())
        return ceres::SOLVER_CONTINUE;
    else
        return ceres::SOLVER_ABORT;
}

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<Simulation>();
    node->run();
}