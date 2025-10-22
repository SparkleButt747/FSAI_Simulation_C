# Created by the script cgal_create_CMakeLists
# This is the CMake script for compiling a set of CGAL applications.

cmake_minimum_required(VERSION 3.10)


# Use C99 and C++17 standards.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Project name and languages.
project(FSAI_Control C CXX)

# CGAL and its components
find_package( CGAL QUIET COMPONENTS  )
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)


if ( NOT CGAL_FOUND )

  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()

endif()


# Boost and its components
find_package( Boost REQUIRED )

if ( NOT Boost_FOUND )

  message(STATUS "This project requires the Boost library, and will not be compiled.")

  return()

endif()


# Creating entries for target: BoundaryEstimation
# ############################

find_package(Eigen3)


# Add system include directory for libyaml.
if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64" OR CMAKE_OSX_ARCHITECTURES MATCHES "arm64")
set(INCLUDE_PATH "/opt/homebrew/include")
set(LINK_PATH "/opt/homebrew/lib")
else()
set(INCLUDE_PATH "/usr/local/include")
set(LINK_PATH "/usr/local/lib")
endif()

include_directories(${INCLUDE_PATH})
link_directories(${LINK_PATH})

include_directories(${CMAKE_SOURCE_DIR}/../sim/track)
include_directories(${CMAKE_SOURCE_DIR}/../common/physics)

# Gather all C source files from the PathLogic folder.
file(GLOB PATH_LOGIC_SOURCES
    "${CMAKE_SOURCE_DIR}/../sim/track/*.cpp"
)

file(GLOB PHYSICS_SOURCES "${CMAKE_SOURCE_DIR}/../common/physics/*.c")


add_executable( BoundaryEstimation BoundaryEstimation.cpp ${PATH_LOGIC_SOURCES} ${PHYSICS_SOURCES})

add_to_cached_list( CGAL_EXECUTABLE_TARGETS BoundaryEstimation )

# Link the executable to CGAL and third-party libraries
# target_link_libraries(BoundaryEstimation PUBLIC Eigen3::Eigen )
find_path(EIGEN3_INCLUDE_DIR Eigen/Core)
target_include_directories(BoundaryEstimation PRIVATE ${EIGEN3_INCLUDE_DIR})
message(${EIGEN3_INCLUDE_DIR})
target_link_libraries(BoundaryEstimation PRIVATE CGAL::CGAL)
if(CGAL_Qt6_FOUND)
  target_link_libraries(BoundaryEstimation PRIVATE CGAL::CGAL_Basic_viewer)
else()
  message(FATAL_ERROR "Qt6 was not found so the triangulation cannot be visualized")
endif()
