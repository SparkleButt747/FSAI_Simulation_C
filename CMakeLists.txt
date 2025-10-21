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

# yaml-cpp (C++ YAML library)
find_package(yaml-cpp REQUIRED)  # provides yaml-cpp::yaml-cpp


# ------------------------------------------------------------------------------
# Optionally silence SDL_main shenanigans if you ever link SDL2main (not used here).
add_definitions(-DSDL_MAIN_HANDLED)

# ------------------------------------------------------------------------------
# Project headers (local)
# ------------------------------------------------------------------------------
set(PROJ_INCLUDE_DIRS
  "${CMAKE_SOURCE_DIR}"
  "${CMAKE_SOURCE_DIR}/tools/Graphics"
  "${CMAKE_SOURCE_DIR}/tools"
  "${CMAKE_SOURCE_DIR}/common/physics"
  "${CMAKE_SOURCE_DIR}/common/time"
  "${CMAKE_SOURCE_DIR}/common/telemetry"
  "${CMAKE_SOURCE_DIR}/control/include"
  "${CMAKE_SOURCE_DIR}/control/include/control"
  "${CMAKE_SOURCE_DIR}/control/src"
  "${CMAKE_SOURCE_DIR}/sim/include"
  "${CMAKE_SOURCE_DIR}/sim/include/io"
  "${CMAKE_SOURCE_DIR}/sim/include/sim"
  "${CMAKE_SOURCE_DIR}/sim/include/sim/integration"
  "${CMAKE_SOURCE_DIR}/sim/src"
  "${CMAKE_SOURCE_DIR}/sim/src/track"
  "${CMAKE_SOURCE_DIR}/sim/src/vehicle"
  "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/include"
)

# ------------------------------------------------------------------------------
# Sources
# ------------------------------------------------------------------------------

# Vehicle models
file(GLOB DYNAMIC_BICYCLE_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/src/vehicle/*.c"
  "${CMAKE_SOURCE_DIR}/sim/src/vehicle/*.cpp"
)
file(GLOB KEY_SOURCES        "${CMAKE_SOURCE_DIR}/tools/KeyboardInputHandler.c")
file(GLOB GRAPHICS_SOURCES   "${CMAKE_SOURCE_DIR}/tools/Graphics/*.c")
file(GLOB GRAPHICS_H_SOURCES "${CMAKE_SOURCE_DIR}/tools/Graphics/*.h")

set(SIMULATION_TEST_SOURCE "${CMAKE_SOURCE_DIR}/sim/src/vehicle/test/TestDynamicBicycle.cpp")

# Path/track logic
file(GLOB PATH_LOGIC_SOURCES "${CMAKE_SOURCE_DIR}/sim/src/track/*.cpp")
set(PATH_LOGIC_TEST_SOURCE "${CMAKE_SOURCE_DIR}/sim/src/track/test/TestTrackGeneration.cpp")

# Main sim
file(GLOB SIM_MAIN_SOURCES "${CMAKE_SOURCE_DIR}/sim/src/*.cpp")
file(GLOB SIM_INTEGRATION_SOURCES "${CMAKE_SOURCE_DIR}/sim/src/integration/*.cpp")
file(GLOB WORLD_SOURCES "${CMAKE_SOURCE_DIR}/sim/src/*.c")
file(GLOB PHYSICS_SOURCES "${CMAKE_SOURCE_DIR}/common/physics/*.c")
file(GLOB_RECURSE CONTROL_CORE_SOURCES
  "${CMAKE_SOURCE_DIR}/control/src/*.cpp"
  "${CMAKE_SOURCE_DIR}/control/src/*.c"
)
set(CONTROL_ALGO_SOURCES "${CMAKE_SOURCE_DIR}/control/src/algorithms/racing_algorithm.c")
file(GLOB SIMULATIONMODELS_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/src/vehicle/*.c"
  "${CMAKE_SOURCE_DIR}/sim/src/vehicle/*.cpp"
)
file(GLOB TELEMETRY_SOURCES
  "${CMAKE_SOURCE_DIR}/common/telemetry/*.c"
  "${CMAKE_SOURCE_DIR}/common/telemetry/*.cpp"
)
file(GLOB TIME_SOURCES "${CMAKE_SOURCE_DIR}/common/time/*.cpp")

# Remove test files from app builds
list(REMOVE_ITEM PATH_LOGIC_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/src/track/test/TestTrackGeneration.cpp")
list(REMOVE_ITEM SIMULATIONMODELS_SOURCES
  "${CMAKE_SOURCE_DIR}/sim/src/vehicle/test/TestDynamicBicycle.cpp")

set(SIMULATION_SOURCE_FINAL "${CMAKE_SOURCE_DIR}/sim/src/main.cpp")
list(REMOVE_ITEM SIM_MAIN_SOURCES "${SIMULATION_SOURCE_FINAL}")

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
    yaml-cpp::yaml-cpp
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
  ${SIM_MAIN_SOURCES}
  ${SIM_INTEGRATION_SOURCES}
  ${WORLD_SOURCES}
  ${PATH_LOGIC_SOURCES}
  ${PHYSICS_SOURCES}
  ${CONTROL_CORE_SOURCES}
  ${SIMULATIONMODELS_SOURCES}
  ${TELEMETRY_SOURCES}
  ${TIME_SOURCES}
  ${KEY_SOURCES}
  ${GRAPHICS_SOURCES}
  ${GRAPHICS_H_SOURCES}
)
target_include_directories(MainController PRIVATE
  ${PROJ_INCLUDE_DIRS}
  ${SDL2_ALL_INCLUDE_DIRS}   # <- adds both .../include/SDL2 and .../include
)
if(TARGET SDL2::SDL2)
  target_link_libraries(MainController PRIVATE Eigen3::Eigen yaml-cpp::yaml-cpp SDL2::SDL2)
else()
  target_link_libraries(MainController PRIVATE Eigen3::Eigen yaml-cpp::yaml-cpp PkgConfig::SDL2)
endif()

# ThrottleTest
set(THROTTLE_TEST_SOURCE "${CMAKE_SOURCE_DIR}/planning/test/TestThrottle.c")
add_executable(ThrottleTest
  ${THROTTLE_TEST_SOURCE}
  ${CONTROL_ALGO_SOURCES}
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
  ${CONTROL_ALGO_SOURCES}
  ${PHYSICS_SOURCES}
)
target_include_directories(SteeringTest PRIVATE ${PROJ_INCLUDE_DIRS})
if(UNIX)
  target_link_libraries(SteeringTest PRIVATE Eigen3::Eigen m)
else()
  target_link_libraries(SteeringTest PRIVATE Eigen3::Eigen)
endif()

