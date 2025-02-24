cmake_minimum_required(VERSION 3.5)
project(dog_interface)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(UNITREE_SRC_DIR "/root/ros2_ws/src/unitree_ros2/example/src") # only source I could find was in the examples subrepo

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

include_directories(${UNITREE_SRC_DIR}/include ${UNITREE_SRC_DIR}/include/common ${UNITREE_SRC_DIR}/include/nlohmann)
link_directories(src)

add_executable(dog_interface src/dog-interface.cpp ${UNITREE_SRC_DIR}/src/common/ros2_sport_client.cpp) # yes there is meant to be src/src in the path dont question it
add_executable(dog_stand src/dog-stand.cpp ${UNITREE_SRC_DIR}/src/common/ros2_sport_client.cpp) # yes there is meant to be src/src in the path dont question it
add_executable(dog_crouch src/dog-crouch.cpp ${UNITREE_SRC_DIR}/src/common/ros2_sport_client.cpp) # yes there is meant to be src/src in the path dont question it
add_executable(dog_sit src/dog-sit.cpp ${UNITREE_SRC_DIR}/src/common/ros2_sport_client.cpp) # yes there is meant to be src/src in the path dont question it

set (
  DEPENDENCY_LIST
  unitree_go
  unitree_hg
  unitree_api
  rclcpp
  std_msgs
  rosbag2_cpp
)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(unitree_go REQUIRED)
find_package(unitree_hg REQUIRED)
find_package(unitree_api REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosbag2_cpp REQUIRED)

ament_target_dependencies(dog_interface ${DEPENDENCY_LIST})
ament_target_dependencies(dog_stand ${DEPENDENCY_LIST})
ament_target_dependencies(dog_crouch ${DEPENDENCY_LIST})
ament_target_dependencies(dog_sit ${DEPENDENCY_LIST})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()
# Install the binary from the external build

install(TARGETS dog_interface # Replace with your target name
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dog_stand # Replace with your target name
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dog_crouch # Replace with your target name
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS dog_sit # Replace with your target name
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
