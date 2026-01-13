cmake_minimum_required(VERSION 3.8)
project(capra_tank_twist)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(THIS_PROJECT_DEPEND
  rclcpp
  geometry_msgs
  sensor_msgs
)

# find dependencies
find_package(ament_cmake REQUIRED)
foreach(dep IN ITEMS ${THIS_PROJECT_DEPEND})
  find_package(${dep} REQUIRED)
endforeach(dep)

add_executable(tank_twist src/tank_twist_node.cpp)
ament_target_dependencies(tank_twist
  ${THIS_PROJECT_DEPEND}
)
target_compile_features(tank_twist PRIVATE cxx_std_20)

install(TARGETS
  tank_twist
  DESTINATION lib/${PROJECT_NAME}
)

# install(DIRECTORY
#   config include
#   DESTINATION share/${PROJECT_NAME}
# )

ament_package()
