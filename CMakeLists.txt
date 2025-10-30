cmake_minimum_required(VERSION 3.20)
project(FSAI_Simulation_C LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(cmake/Options.cmake)

if(APPLE)
  include(cmake/PlatformMac.cmake)
else()
  include(cmake/PlatformLinux.cmake)
endif()

add_definitions(-DSDL_MAIN_HANDLED)

find_package(PkgConfig REQUIRED)

find_package(Eigen3 3.3 QUIET NO_MODULE)
if(NOT Eigen3_FOUND)
  pkg_check_modules(EIGEN3 REQUIRED eigen3)
  add_library(Eigen3::Eigen INTERFACE IMPORTED)
  set_target_properties(Eigen3::Eigen PROPERTIES
    INTERFACE_INCLUDE_DIRECTORIES "${EIGEN3_INCLUDE_DIRS}"
  )
endif()
find_package(Threads REQUIRED)
find_package(OpenGL REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(CGAL REQUIRED)

set(SDL2_IMPORTED_TARGET "")
find_package(SDL2 QUIET CONFIG)
if(TARGET SDL2::SDL2)
  set(SDL2_IMPORTED_TARGET SDL2::SDL2)
elseif(TARGET SDL2::SDL2-static)
  set(SDL2_IMPORTED_TARGET SDL2::SDL2-static)
else()
  pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
  set(SDL2_IMPORTED_TARGET PkgConfig::SDL2)
endif()

if(SDL2_IMPORTED_TARGET)
  get_target_property(SDL2_INCLUDE_DIRS ${SDL2_IMPORTED_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
endif()

if(FSAI_ENABLE_TESTS)
  enable_testing()
endif()

add_subdirectory(third_party/glad)
add_subdirectory(third_party/imgui)
add_subdirectory(common)
add_subdirectory(io)
add_subdirectory(sim)
add_subdirectory(control)

if(FSAI_ENABLE_APPS)
  add_subdirectory(apps)
endif()

if(FSAI_BUILD_VISION AND EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/vision/CMakeLists.txt")
  add_subdirectory(vision)
endif()
