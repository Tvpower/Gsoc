cmake_minimum_required(VERSION 3.8)
project(my_rfid_demo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

#dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
find_package(std_msgs REQUIRED)

#Gazebo Harmonic plugins
find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED)
find_package(gz-transport13 REQUIRED)
find_package(gz-msgs10 REQUIRED)

#dummy sensor plugin library
add_library(dummy_sensor SHARED
  src/plugins/dummy_sensor.cpp
  include/my_rfid_demo/dummy_sensor.hpp

)

target_include_directories(dummy_sensor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(dummy_sensor
  gz-sim8::gz-sim8
  gz-plugin2::gz-plugin2
  gz-transport13::gz-transport13
  gz-msgs10::gz-msgs10
)

#install the plugin library
install(TARGETS dummy_sensor
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

#install include directories
install(DIRECTORY include/
  DESTINATION include/
)

#install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

#install worlds
install(DIRECTORY
  worlds
  DESTINATION share/${PROJECT_NAME}
)

# install urdf
install(DIRECTORY
  urdf
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
