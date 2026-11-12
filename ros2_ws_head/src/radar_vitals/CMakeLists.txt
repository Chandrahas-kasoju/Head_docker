cmake_minimum_required(VERSION 3.8)
project(radar_vitals)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/VitalSigns.msg"
  DEPENDENCIES std_msgs
)

ament_python_install_package(radar_vitals_core)

install(PROGRAMS
  scripts/radar_vitals_node
  scripts/replay_recording
  scripts/breathing_plot
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY cfg launch rviz DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(radar_vitals_tests test)
endif()

ament_package()
