cmake_minimum_required(VERSION 3.18)
project(semantic_mapping LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

add_library(semantic_layer SHARED src/semantic_layer.cpp)
ament_target_dependencies(semantic_layer nav2_costmap_2d nav_msgs pluginlib rclcpp std_msgs)
pluginlib_export_plugin_description_file(nav2_costmap_2d semantic_layer.xml)
install(TARGETS semantic_layer LIBRARY DESTINATION lib)

set(ORB_SLAM3_ROOT "" CACHE PATH "Path to the built, patched ORB-SLAM3 source tree")
if(ORB_SLAM3_ROOT)
  find_package(cv_bridge REQUIRED)
  find_package(Eigen3 REQUIRED)
  find_package(geometry_msgs REQUIRED)
  find_package(OpenCV REQUIRED COMPONENTS core imgproc features2d calib3d)
  find_package(Pangolin REQUIRED)
  find_package(sensor_msgs REQUIRED)
  find_package(tf2 REQUIRED)
  find_package(tf2_ros REQUIRED)
  find_library(ORB_SLAM3_LIBRARY ORB_SLAM3 PATHS "${ORB_SLAM3_ROOT}/lib" NO_DEFAULT_PATH REQUIRED)
  add_executable(stereo_inertial src/stereo_inertial.cpp)
  target_include_directories(stereo_inertial PRIVATE
    "${ORB_SLAM3_ROOT}" "${ORB_SLAM3_ROOT}/include"
    "${ORB_SLAM3_ROOT}/include/CameraModels" "${ORB_SLAM3_ROOT}/Thirdparty/Sophus"
    ${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS})
  target_link_libraries(stereo_inertial
    "${ORB_SLAM3_LIBRARY}" Eigen3::Eigen ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
  ament_target_dependencies(stereo_inertial
    cv_bridge geometry_msgs rclcpp sensor_msgs std_msgs tf2 tf2_ros)
  install(TARGETS stereo_inertial DESTINATION lib/${PROJECT_NAME})

  include(CTest)
  if(BUILD_TESTING)
    add_executable(test_orb_mask test/orb_mask.cpp)
    target_include_directories(test_orb_mask PRIVATE "${ORB_SLAM3_ROOT}/include" ${OpenCV_INCLUDE_DIRS})
    target_link_libraries(test_orb_mask "${ORB_SLAM3_LIBRARY}" ${OpenCV_LIBS})
    add_test(NAME orb_mask_excludes_features COMMAND test_orb_mask)
  endif()
endif()

ament_python_install_package(${PROJECT_NAME})
install(PROGRAMS scripts/segmentation scripts/semantic_fusion scripts/velocity_gate
  DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
ament_package()
