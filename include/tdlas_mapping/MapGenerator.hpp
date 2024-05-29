#pragma once
#include <vector>
#include <tdlas_mapping/common.h>
#include <ceres/ceres.h>
#include <ceres/dynamic_autodiff_cost_function.h>
#include <DDA/DDA.h>
#include <opencv4/opencv2/core.hpp>

struct MapGeneratorOptions
{
    int numCells;
    int numMeasurements;
    double prior;
    bool useRayPrior;
    double lambda;
};

struct TDLASMeasurement
{
    glm::vec2 origin;
    glm::vec2 reflectorPosition;
    glm::vec2 direction;
    float ppmxm;
};

class MapGenerator
{
public:
    MapGenerator(const MapGeneratorOptions& options, std::vector<double>& concentrations);
    void SetMap(const std::vector<std::vector<bool>>& occupancy_map, glm::vec2 mapOrigin, float rayMarchResolution);
    void SetMeasurements(const std::vector<TDLASMeasurement>& measurements);
    std::vector<double> Solve(const std::vector<ceres::IterationCallback*>& callbacks);
    void WriteHeatmap();

private: 
    void runDDA(const glm::vec2& origin, const glm::vec2& direction, const glm::vec2& reflectorPosition, uint rowIndex, float ppmxm);
    uint index2Dto1D(const glm::ivec2& index)
    {
        int num_cells_x = m_occupancy_map.size();
        int num_cells_y = m_occupancy_map[0].size();
	    return index.x + index.y*num_cells_x;
    }

    MapGeneratorOptions m_options;
    std::vector<double>& m_concentration; //(num_cells);
    std::vector<RollingAverage> m_concentrationPrior; //(num_cells);
    std::vector<double> m_measurements; //(num_measurements);
    std::vector<std::vector<double>> m_lengthRayInCell; //(num_measurements, num_cells);
    std::vector<std::vector<bool>> m_occupancy_map;
    float m_rayMarchResolution;
    glm::vec2 m_mapOrigin;

    cv::Mat m_raysImage;
};


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
        T backgroundC{1.9}; //background concentration of methane in the atmosphere, according to wikipedia
        T sum{0};
        for (int i = 0; i < size; i++)
        {
            T error = array[i] - backgroundC;
            sum += error * error;
            // sum += ceres::abs(array[i] -T{i}); //test
        }
        // return ceres::sqrt(sum+1e-8);
        return sum;
    }
};
