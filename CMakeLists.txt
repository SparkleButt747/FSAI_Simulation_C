cmake_minimum_required(VERSION 3.10)

# ------------------------------------------------------------------------------
# Project
# ------------------------------------------------------------------------------
project(FSAI_Simulation LANGUAGES C CXX)

# Standards
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Host / toolchain defaults (don’t hard-code compilers)
# ------------------------------------------------------------------------------
if(APPLE AND NOT DEFINED CMAKE_OSX_ARCHITECTURES)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "" FORCE)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "" FORCE)
  endif()
endif()

# Help CMake find Homebrew-installed packages on both ARM (/opt/homebrew) and Intel (/usr/local).
if(APPLE)
  find_program(BREW brew PATHS /opt/homebrew/bin /usr/local/bin /usr/bin NO_DEFAULT_PATH)
  if(BREW)
    execute_process(COMMAND "${BREW}" --prefix
      OUTPUT_VARIABLE HOMEBREW_PREFIX
      OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(HOMEBREW_PREFIX AND EXISTS "${HOMEBREW_PREFIX}")
      list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}")
      # Also hint pkg-config (helps some setups)
      set(ENV{PKG_CONFIG_PATH} "${HOMEBREW_PREFIX}/lib/pkgconfig:${HOMEBREW_PREFIX}/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
    endif()
  endif()
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------

# Eigen3
find_package(Eigen3 3.3 REQUIRED NO_MODULE)

# pkg-config for SDL2 and libyaml
find_package(PkgConfig REQUIRED)

# SDL2: prefer CMake package, fall back to pkg-config IMPORTED target
find_package(SDL2 QUIET CONFIG)
if(NOT SDL2_FOUND)
  pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
  # Provides PkgConfig::SDL2
endif()

# libyaml (pkg-config name: yaml-0.1) via IMPORTED target
pkg_check_modules(YAML REQUIRED IMPORTED_TARGET yaml-0.1)
# Provides PkgConfig::YAML

# ---- SDL2 include compatibility ---------------------------------------------
# Many projects include <SDL2/SDL.h>. pkg-config typically emits -I.../include/SDL2.
# That breaks <SDL2/SDL.h>. Add the *parent* include dir (…/include) as well.
set(SDL2_ALL_INCLUDE_DIRS "")
if(TARGET SDL2::SDL2)
  get_target_property(_sdl2_inc SDL2::SDL2 INTERFACE_INCLUDE_DIRECTORIES)
elseif(TARGET PkgConfig::SDL2)
  get_target_property(_sdl2_inc PkgConfig::SDL2 INTERFACE_INCLUDE_DIRECTORIES)
endif()
if(_sdl2_inc)
  foreach(d IN LISTS _sdl2_inc)
    if(d)
      list(APPEND SDL2_ALL_INCLUDE_DIRS "${d}")
      get_filename_component(_parent "${d}" DIRECTORY)
      list(APPEND SDL2_ALL_INCLUDE_DIRS "${_parent}")
    endif()
  endforeach()
  list(REMOVE_DUPLICATES SDL2_ALL_INCLUDE_DIRS)
endif()
# ------------------------------------------------------------------------------
# Optionally silence SDL_main shenanigans if you ever link SDL2main (not used here).
add_definitions(-DSDL_MAIN_HANDLED)

# ------------------------------------------------------------------------------
# Project headers (local)
# ------------------------------------------------------------------------------
set(PROJ_INCLUDE_DIRS
  "${CMAKE_SOURCE_DIR}/tools/Graphics"
  "${CMAKE_SOURCE_DIR}/tools"
  "${CMAKE_SOURCE_DIR}/MainController"
  "${CMAKE_SOURCE_DIR}/sim"
  "${CMAKE_SOURCE_DIR}/sim/track"
  "${CMAKE_SOURCE_DIR}/common/physics"
  "${CMAKE_SOURCE_DIR}/planning"
  "${CMAKE_SOURCE_DIR}/sim/vehicle"
  "${CMAKE_SOURCE_DIR}/common/telemetry"
)

# ------------------------------------------------------------------------------
# Sources
# ------------------------------------------------------------------------------

# Vehicle models
file(GLOB DYNAMIC_BICYCLE_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/vehicle/*.c"
  "${CMAKE_SOURCE_DIR}/sim/vehicle/*.cpp"
)
file(GLOB KEY_SOURCES        "${CMAKE_SOURCE_DIR}/tools/KeyboardInputHandler.c")
file(GLOB GRAPHICS_SOURCES   "${CMAKE_SOURCE_DIR}/tools/Graphics/*.c")
file(GLOB GRAPHICS_H_SOURCES "${CMAKE_SOURCE_DIR}/tools/Graphics/*.h")

set(SIMULATION_TEST_SOURCE "${CMAKE_SOURCE_DIR}/sim/vehicle/test/TestDynamicBicycle.cpp")

# Path/track logic
file(GLOB PATH_LOGIC_SOURCES "${CMAKE_SOURCE_DIR}/sim/track/*.cpp")
set(PATH_LOGIC_TEST_SOURCE "${CMAKE_SOURCE_DIR}/sim/track/test/TestTrackGeneration.cpp")

# Main sim
file(GLOB MAINCONTROLLER_SOURCES
  "${CMAKE_SOURCE_DIR}/MainController/*.c"
  "${CMAKE_SOURCE_DIR}/MainController/*.cpp"
)
file(GLOB WORLD_SOURCES "${CMAKE_SOURCE_DIR}/sim/*.cpp")
file(GLOB PHYSICS_SOURCES "${CMAKE_SOURCE_DIR}/common/physics/*.c")
file(GLOB RACING_SOURCES  "${CMAKE_SOURCE_DIR}/planning/*.c")
file(GLOB SIMULATIONMODELS_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/vehicle/*.c"
  "${CMAKE_SOURCE_DIR}/sim/vehicle/*.cpp"
)
file(GLOB TELEMETRY_SOURCES
  "${CMAKE_SOURCE_DIR}/common/telemetry/*.c"
  "${CMAKE_SOURCE_DIR}/common/telemetry/*.cpp"
)

# Remove test files from app builds
list(REMOVE_ITEM PATH_LOGIC_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/track/test/TestTrackGeneration.cpp")
list(REMOVE_ITEM SIMULATIONMODELS_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/vehicle/test/TestDynamicBicycle.cpp")

set(SIMULATION_SOURCE_FINAL "${CMAKE_SOURCE_DIR}/MainController/main.cpp")

# ------------------------------------------------------------------------------
# Executables
# ------------------------------------------------------------------------------

# SimulationModelsTest
add_executable(SimulationModelsTest
  ${SIMULATION_TEST_SOURCE}
  ${DYNAMIC_BICYCLE_SOURCES}
)
target_include_directories(SimulationModelsTest PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(SimulationModelsTest
  PRIVATE
    Eigen3::Eigen
    PkgConfig::YAML
)

# PathLogicTest
add_executable(PathLogicTest
  ${PATH_LOGIC_TEST_SOURCE}
  ${PATH_LOGIC_SOURCES}
)
target_include_directories(PathLogicTest PRIVATE ${PROJ_INCLUDE_DIRS})
target_link_libraries(PathLogicTest PRIVATE Eigen3::Eigen)

# MainController
add_executable(MainController
  ${SIMULATION_SOURCE_FINAL}
  ${MAINCONTROLLER_SOURCES}
  ${WORLD_SOURCES}
  ${PATH_LOGIC_SOURCES}
  ${PHYSICS_SOURCES}
  ${RACING_SOURCES}
  ${SIMULATIONMODELS_SOURCES}
  ${TELEMETRY_SOURCES}
  ${KEY_SOURCES}
  ${GRAPHICS_SOURCES}
  ${GRAPHICS_H_SOURCES}
)
target_include_directories(MainController PRIVATE
  ${PROJ_INCLUDE_DIRS}
  ${SDL2_ALL_INCLUDE_DIRS}   # <- adds both .../include/SDL2 and .../include
)
if(TARGET SDL2::SDL2)
  target_link_libraries(MainController PRIVATE Eigen3::Eigen PkgConfig::YAML SDL2::SDL2)
else()
  target_link_libraries(MainController PRIVATE Eigen3::Eigen PkgConfig::YAML PkgConfig::SDL2)
endif()

# ThrottleTest
set(THROTTLE_TEST_SOURCE "${CMAKE_SOURCE_DIR}/planning/test/TestThrottle.c")
add_executable(ThrottleTest
  ${THROTTLE_TEST_SOURCE}
  ${RACING_SOURCES}
  ${PHYSICS_SOURCES}
)
target_include_directories(ThrottleTest PRIVATE ${PROJ_INCLUDE_DIRS})
if(UNIX)
  target_link_libraries(ThrottleTest PRIVATE Eigen3::Eigen m)
else()
  target_link_libraries(ThrottleTest PRIVATE Eigen3::Eigen)
endif()

# SteeringTest
set(STEERING_TEST_SOURCE "${CMAKE_SOURCE_DIR}/planning/test/TestSteering.c")
add_executable(SteeringTest
  ${STEERING_TEST_SOURCE}
  ${RACING_SOURCES}
  ${PHYSICS_SOURCES}
)
target_include_directories(SteeringTest PRIVATE ${PROJ_INCLUDE_DIRS})
if(UNIX)
  target_link_libraries(SteeringTest PRIVATE Eigen3::Eigen m)
else()
  target_link_libraries(SteeringTest PRIVATE Eigen3::Eigen)
endif()
