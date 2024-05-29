cmake_minimum_required(VERSION 3.5)
project(tdlas_mapping)

##################
set(CMAKE_BUILD_TYPE "Release") 
set(CMAKE_CXX_FLAGS_RELEASE "-std=c++17 -O3 ${CMAKE_CXX_FLAGS}")
set(CMAKE_CXX_FLAGS_DEBUG "-std=c++17 -g -O0 ${CMAKE_CXX_FLAGS}")



find_package(ament_cmake REQUIRED)
set(AMENT_DEPS
    rclcpp
    rclcpp_action
    std_msgs
    visualization_msgs
    tf2_geometry_msgs
    nav2_msgs
    ament_imgui
    rosgraph_msgs
    diagnostic_msgs
    olfaction_msgs
    image_marker_msgs
    )

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()

find_package(Ceres)


include_directories(include include/DDA/third_party/glm)


#Log Measurements
add_executable(log_measurements src/log_measurements.cpp)
ament_target_dependencies(log_measurements
${AMENT_DEPS}
) 

#Generate map
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(fmt REQUIRED)
add_executable(generate_map src/generate_map.cpp src/MapGenerator.cpp)
target_include_directories(generate_map PRIVATE include/DDA/include)
target_link_libraries(generate_map 
    ${OpenCV_LIBS} 
    fmt 
    OpenMP::OpenMP_CXX
    Ceres::ceres)
ament_target_dependencies(generate_map
${AMENT_DEPS}
) 

#Fake map from simulations
find_package(OpenCV REQUIRED)
find_package(OpenMP REQUIRED)
find_package(fmt REQUIRED)
add_executable(map_simulations src/mapFromSimulations.cpp src/MapGenerator.cpp)
target_include_directories(map_simulations PRIVATE include/DDA/include)
target_link_libraries(map_simulations 
    ${OpenCV_LIBS} 
    fmt 
    OpenMP::OpenMP_CXX
    Ceres::ceres)
ament_target_dependencies(map_simulations
${AMENT_DEPS}
) 

#Trajectory painter
add_executable(paint_trajectory src/paint_trajectory.cpp)
ament_target_dependencies(paint_trajectory
${AMENT_DEPS}
) 


install(
  TARGETS log_measurements generate_map paint_trajectory map_simulations
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch maps
  DESTINATION share/${PROJECT_NAME}
)
ament_package()