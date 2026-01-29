cmake_minimum_required(VERSION 3.8)
project(so_arm100_kinematics)

# if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
#   add_compile_options(-Wall -Wextra -Wpedantic)
# endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(moveit_core REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Eigen3 REQUIRED)

if(NOT DEFINED CMAKE_EXPORT_COMPILE_COMMANDS)
  set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
endif()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)

  # Add all unit test files from test folder
  file(GLOB TEST_FILES test/*Test.cpp)
  foreach(TEST_FILE ${TEST_FILES})
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    ament_add_gtest(${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${TEST_NAME} ${PROJECT_NAME} Eigen3::Eigen)
  endforeach()

  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  ament_uncrustify(
    CONFIG_FILE ${CMAKE_CURRENT_SOURCE_DIR}/uncrustify.cfg
  )
  endif()
  
include_directories(include)
ament_export_include_directories(include)

# Add all source files from src folder
file(GLOB SOURCE_FILES src/*.cpp)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

ament_target_dependencies(${PROJECT_NAME}
                          pluginlib
                          moveit_core
                          rclcpp
                          Eigen3)

pluginlib_export_plugin_description_file( kinematics_base so_arm100_kinematics_plugin.xml )

ament_export_libraries(so_arm100_kinematics_plugin)

install(TARGETS ${PROJECT_NAME}
        LIBRARY DESTINATION share/${PROJECT_NAME}/lib)

ament_package()