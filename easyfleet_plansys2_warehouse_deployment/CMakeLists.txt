cmake_minimum_required(VERSION 3.20)
project(easyfleet_plansys2_warehouse_deployment)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(easyfleet_mission_manager REQUIRED)
find_package(plansys2_domain_expert REQUIRED)
find_package(plansys2_executor REQUIRED)
find_package(plansys2_msgs REQUIRED)
find_package(plansys2_planner REQUIRED)
find_package(plansys2_problem_expert REQUIRED)

add_executable(warehouse_controller src/warehouse_controller.cpp)
target_link_libraries(warehouse_controller
  rclcpp::rclcpp
  easyfleet_mission_manager::easyfleet_mission_manager
  plansys2_domain_expert::plansys2_domain_expert
  plansys2_executor::plansys2_executor
  ${plansys2_msgs_TARGETS}
  plansys2_planner::plansys2_planner
  plansys2_problem_expert::plansys2_problem_expert
)

install(
  TARGETS warehouse_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config pddl behavior_trees rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
