cmake_minimum_required(VERSION 3.8)
project(so_arm100_kinematics)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 23)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
  set(CMAKE_CXX_EXTENSIONS OFF) # Set to ON if you need GNU extensions (std=gnu++23)
endif()
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

# Add all source files from src folder
file(GLOB SOURCE_FILES src/*.cpp)

add_library(${PROJECT_NAME} ${SOURCE_FILES})

if(BUILD_TESTING)
find_package(ament_cmake_gtest REQUIRED)
find_package(ament_cmake_cppcheck REQUIRED)
find_package(ament_cmake_uncrustify REQUIRED)
find_package(code_style REQUIRED)

  file(GLOB INCLUDE_FILES include/*.hpp)
  file(GLOB TEST_FILES test/*Test.cpp)
  file(GLOB TEST_INCLUDE_FILES test/include/*.hpp)

  set(ALL_LINT_FILES
    ${INCLUDE_FILES}
    ${SOURCE_FILES}
    ${TEST_FILES}
    ${TEST_INCLUDE_FILES}
  )
  # ------------------------------------------------------------
  # Test utility library (shared test code)
  # ------------------------------------------------------------

  add_library(${PROJECT_NAME}_test_utils STATIC
    test/src/RobotModelTestData.cpp
  )

  target_include_directories(${PROJECT_NAME}_test_utils PUBLIC
    test/include
  )
  
  target_link_libraries(${PROJECT_NAME}_test_utils
    ${PROJECT_NAME}
    Eigen3::Eigen
  )
  
  # ------------------------------------------------------------
  # Test executables
  # ------------------------------------------------------------

  # Add all unit test files from test folder
  foreach(TEST_FILE ${TEST_FILES})

    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)

    ament_add_gtest(${TEST_NAME} ${TEST_FILE})

    set_target_properties(${TEST_NAME} PROPERTIES
      RUNTIME_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/test
    )

    target_link_libraries(${TEST_NAME}
      ${PROJECT_NAME}_test_utils 
      Eigen3::Eigen)

  endforeach()

  # ------------------------------------------------------------
  # cpp_check
  # ------------------------------------------------------------

  ament_cppcheck( ${ALL_LINT_FILES} )

  # ------------------------------------------------------------
  # Uncrustify
  # ------------------------------------------------------------

  set(UNCRUSTIFY_CONFIG_FILE "${code_style_DIR}/../uncrustify.cfg")

  # Add an ament_uncrustify test explicitly
  ament_uncrustify(
    TARGET so_arm100_kinematics_uncrustify
    CONFIG_FILE "${UNCRUSTIFY_CONFIG_FILE}"
    FILES ${ALL_LINT_FILES}
  )

endif()

include_directories(include)
ament_export_include_directories(include)

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