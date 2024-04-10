cmake_minimum_required(VERSION 3.5)
project(tdlas_mapping)

##################
set(CMAKE_BUILD_TYPE "None") 
set(CMAKE_CXX_FLAGS "-std=c++17 -g -O0 ${CMAKE_CXX_FLAGS}")



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
    )

foreach(Dependency IN ITEMS ${AMENT_DEPS})
  find_package(${Dependency} REQUIRED)
endforeach()



include_directories(include)


#test client
add_executable(test_node src/test.cpp)
ament_target_dependencies(test_node
${AMENT_DEPS}
) 
    

#MASTER
add_executable(reactive_master src/reactive.cpp src/reactive_master.cpp)
ament_target_dependencies(reactive_master
${AMENT_DEPS}
) 

#FOLLOWER
add_executable(reactive_follower src/reactive_follower.cpp)
ament_target_dependencies(reactive_follower
${AMENT_DEPS}
) 

#CLOCK SERVER (currently unused)
add_executable(clock_server src/clock_server.cpp)
ament_target_dependencies(clock_server
${AMENT_DEPS}
) 

#Log Measurements
add_executable(log_measurements src/log_measurements.cpp)
ament_target_dependencies(log_measurements
${AMENT_DEPS}
) 

#Generate map
find_package(OpenCV REQUIRED)
find_package(fmt REQUIRED)
add_executable(generate_map src/generate_map.cpp)
target_include_directories(generate_map PRIVATE include/DDA/include)
target_link_libraries(generate_map ${OpenCV_LIBS} fmt)
ament_target_dependencies(generate_map
${AMENT_DEPS}
) 

#Trajectory painter
add_executable(paint_trajectory src/paint_trajectory.cpp)
ament_target_dependencies(paint_trajectory
${AMENT_DEPS}
) 


install(
  TARGETS test_node reactive_master reactive_follower clock_server log_measurements generate_map paint_trajectory
  DESTINATION lib/${PROJECT_NAME}
)
install(
  DIRECTORY launch maps data
  DESTINATION share/${PROJECT_NAME}
)
ament_package()