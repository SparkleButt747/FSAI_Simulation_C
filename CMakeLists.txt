cmake_minimum_required(VERSION 3.20)
project(FSAI_Simulation_C LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

include(CTest)
include(cmake/Options.cmake)

if(APPLE)
  include(cmake/PlatformMac.cmake)
elseif(UNIX)
  include(cmake/PlatformLinux.cmake)
endif()

include(FetchContent)

find_path(EIGEN3_INCLUDE_DIR Eigen/Core)
if(NOT EIGEN3_INCLUDE_DIR)
  message(STATUS "Eigen3 not found; fetching 3.4.0")
  FetchContent_Declare(
    eigen
    URL https://gitlab.com/libeigen/eigen/-/archive/3.4.0/eigen-3.4.0.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_GetProperties(eigen)
  if(NOT eigen_POPULATED)
    FetchContent_Populate(eigen)
  endif()
  set(EIGEN3_INCLUDE_DIR ${eigen_SOURCE_DIR})
endif()

if(NOT TARGET Eigen3::Eigen)
  add_library(Eigen3_Eigen INTERFACE)
  target_include_directories(Eigen3_Eigen INTERFACE ${EIGEN3_INCLUDE_DIR})
  add_library(Eigen3::Eigen ALIAS Eigen3_Eigen)
endif()

find_package(Threads REQUIRED)
find_package(OpenGL QUIET)
if(NOT OpenGL_FOUND)
  message(WARNING "OpenGL not found; OpenGL-dependent targets will rely on glad loader only")
endif()
find_package(PkgConfig REQUIRED)
find_package(yaml-cpp QUIET)
if(NOT yaml-cpp_FOUND)
  message(STATUS "yaml-cpp not found; fetching 0.8.0")
  FetchContent_Declare(
    yamlcpp
    GIT_REPOSITORY https://github.com/jbeder/yaml-cpp.git
    GIT_TAG yaml-cpp-0.7.0)
  FetchContent_MakeAvailable(yamlcpp)
endif()

find_package(SDL2 QUIET CONFIG)
set(SDL2_IMPORTED_TARGET "")
if(TARGET SDL2::SDL2)
  set(SDL2_IMPORTED_TARGET SDL2::SDL2)
elseif(TARGET SDL2::SDL2-static)
  set(SDL2_IMPORTED_TARGET SDL2::SDL2-static)
endif()
if(SDL2_IMPORTED_TARGET STREQUAL "")
  pkg_check_modules(SDL2 IMPORTED_TARGET sdl2)
  if(SDL2_FOUND)
    set(SDL2_IMPORTED_TARGET PkgConfig::SDL2)
  endif()
endif()
if(SDL2_IMPORTED_TARGET STREQUAL "")
  message(STATUS "SDL2 not found; fetching 2.30.7")
  FetchContent_Declare(
    sdl2
    URL https://github.com/libsdl-org/SDL/releases/download/release-2.30.7/SDL2-2.30.7.tar.gz
    DOWNLOAD_EXTRACT_TIMESTAMP TRUE)
  FetchContent_MakeAvailable(sdl2)
  if(TARGET SDL2::SDL2-static)
    set(SDL2_IMPORTED_TARGET SDL2::SDL2-static)
  elseif(TARGET SDL2::SDL2)
    set(SDL2_IMPORTED_TARGET SDL2::SDL2)
  endif()
  set(FSAI_SDL2_INCLUDE_DIRS ${sdl2_SOURCE_DIR}/include)
endif()
set(FSAI_SDL2_TARGET ${SDL2_IMPORTED_TARGET})
if(FSAI_SDL2_INCLUDE_DIRS)
  # already set from fetch
elseif(DEFINED SDL2_INCLUDE_DIRS)
  set(FSAI_SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS})
elseif(SDL2_IMPORTED_TARGET)
  get_target_property(_sdl_includes ${SDL2_IMPORTED_TARGET} INTERFACE_INCLUDE_DIRECTORIES)
  set(FSAI_SDL2_INCLUDE_DIRS ${_sdl_includes})
endif()

find_package(CGAL QUIET COMPONENTS)
if(CGAL_FOUND)
  set(FSAI_HAVE_CGAL TRUE)
else()
  message(WARNING "CGAL not found; control module will build without CGAL features")
  set(FSAI_HAVE_CGAL FALSE)
endif()

add_definitions(-DSDL_MAIN_HANDLED)

add_subdirectory(third_party)
add_subdirectory(common)
add_subdirectory(io)
add_subdirectory(sim)
add_subdirectory(control)
add_subdirectory(apps)
if(FSAI_BUILD_VISION)
  add_subdirectory(vision)
endif()

if(BUILD_TESTING)
  enable_testing()
endif()
