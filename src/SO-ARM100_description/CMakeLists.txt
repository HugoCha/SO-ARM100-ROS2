cmake_minimum_required(VERSION 3.8)
project(so_arm100_description)

# Check if the compiler being used is GNU's C++ compiler (g++) or Clang.
# Add compiler flags for all targets that will be defined later in the 
# CMakeLists file. These flags enable extra warnings to help catch
# potential issues in the code.
# Add options to the compilation process
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(xacro REQUIRED)

# ---------------------------------------------------------------------------
# XACRO processing (optional — for validation or auto-generation)
# ---------------------------------------------------------------------------
# This is optional; it will just check that your .xacro compiles correctly when building
xacro_add_files(
  urdf/so_arm100.urdf.xacro
  TARGET xacro_urdf
  INSTALL DESTINATION urdf
)

# Install files so they are found via $(find so_arm100_description)
install(DIRECTORY urdf meshes launch rviz
DESTINATION share/${PROJECT_NAME}
)

ament_package()
