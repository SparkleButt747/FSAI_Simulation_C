cmake_minimum_required(VERSION 3.10)

# Project name and languages.
project(FSAI_Simulation C CXX)

# Use C99 and C++17 standards.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Include directories for headers.
include_directories(${CMAKE_SOURCE_DIR}/tools/Graphics)
include_directories(${CMAKE_SOURCE_DIR}/tools)
include_directories(${CMAKE_SOURCE_DIR}/MainController)
include_directories(${CMAKE_SOURCE_DIR}/sim)
include_directories(${CMAKE_SOURCE_DIR}/sim/track)
include_directories(${CMAKE_SOURCE_DIR}/common/physics)
include_directories(${CMAKE_SOURCE_DIR}/planning)
include_directories(${CMAKE_SOURCE_DIR}/sim/vehicle)
include_directories(${CMAKE_SOURCE_DIR}/common/telemetry)
include_directories(${CMAKE_SOURCE_DIR}/common/physics)
find_package(Eigen3 REQUIRED)

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

# Find SDL2
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

# ------------------------------------------------------------------------------
# Build the SimulationModels test executable.
# ------------------------------------------------------------------------------

# Gather all C/C++ source files from the DynamicBicycle folder.
file(GLOB DYNAMIC_BICYCLE_SOURCES "${CMAKE_SOURCE_DIR}/sim/vehicle/*.c" "${CMAKE_SOURCE_DIR}/sim/vehicle/*.cpp")
file(GLOB KEY_SOURCES "${CMAKE_SOURCE_DIR}/tools/KeyboardInputHandler.c")
file(GLOB GRAPHICS_SOURCES "${CMAKE_SOURCE_DIR}/tools/Graphics/*.c")
file(GLOB GRAPHICS_H_SOURCES "${CMAKE_SOURCE_DIR}/tools/Graphics/*.h")

# The test driver for SimulationModels is at SimulationModels/test.cpp.
set(SIMULATION_TEST_SOURCE "${CMAKE_SOURCE_DIR}/sim/vehicle/test/TestDynamicBicycle.cpp")

# Create the SimulationModels executable.
add_executable(SimulationModelsTest ${SIMULATION_TEST_SOURCE} ${DYNAMIC_BICYCLE_SOURCES})

target_link_libraries(SimulationModelsTest Eigen3::Eigen yaml)

# ------------------------------------------------------------------------------
# Build the PathLogic test executable.
# ------------------------------------------------------------------------------

# Gather all C source files from the PathLogic folder.
file(GLOB PATH_LOGIC_SOURCES
    "${CMAKE_SOURCE_DIR}/sim/track/*.cpp"
)

# The test driver for PathLogic is at PathLogic/test.c.
set(PATH_LOGIC_TEST_SOURCE "${CMAKE_SOURCE_DIR}/sim/track/test/TestTrackGeneration.cpp")

# Create the PathLogic executable.
add_executable(PathLogicTest ${PATH_LOGIC_TEST_SOURCE} ${PATH_LOGIC_SOURCES})
target_link_libraries(PathLogicTest Eigen3::Eigen)

# ------------------------------------------------------------------------------
# Build the Main simulation executable.
# ------------------------------------------------------------------------------

# Collect source files.
file(GLOB MAINCONTROLLER_SOURCES "${CMAKE_SOURCE_DIR}/MainController/*.c" "${CMAKE_SOURCE_DIR}/MainController/*.cpp")
file(GLOB WORLD_SOURCES "${CMAKE_SOURCE_DIR}/sim/*.cpp")

#Remove test.c from the list of source files
file(GLOB PATHLOGIC_SOURCES "${CMAKE_SOURCE_DIR}/sim/track/*.cpp")
list(REMOVE_ITEM PATHLOGIC_SOURCES "${CMAKE_SOURCE_DIR}/sim/track/test/TestTrackGeneration.cpp")

file(GLOB PHYSICS_SOURCES "${CMAKE_SOURCE_DIR}/common/physics/*.c")
file(GLOB RACING_SOURCES "${CMAKE_SOURCE_DIR}/planning/*.c")

#Remove test.c from the list of source files
file(GLOB SIMULATIONMODELS_SOURCES "${CMAKE_SOURCE_DIR}/sim/vehicle/*.c" "${CMAKE_SOURCE_DIR}/sim/vehicle/*.cpp")
list(REMOVE_ITEM SIMULATIONMODELS_SOURCES "${CMAKE_SOURCE_DIR}/sim/vehicle/test/TestDynamicBicycle.cpp")

file(GLOB TELEMETRY_SOURCES "${CMAKE_SOURCE_DIR}/common/telemetry/*.c" "${CMAKE_SOURCE_DIR}/common/telemetry/*.cpp")

set(SIMULATION_SOURCE_FINAL "${CMAKE_SOURCE_DIR}/MainController/main.cpp")

add_executable(MainController
    ${SIMULATION_SOURCE_FINAL}
    ${MAINCONTROLLER_SOURCES}
    ${WORLD_SOURCES}
    ${PATHLOGIC_SOURCES}
    ${PHYSICS_SOURCES}
    ${RACING_SOURCES}
    ${SIMULATIONMODELS_SOURCES}
    ${TELEMETRY_SOURCES}
    ${KEY_SOURCES}
    ${GRAPHICS_SOURCES}
    ${GRAPHICS_H_SOURCES}
)

target_link_libraries(MainController Eigen3::Eigen yaml ${SDL2_LIBRARIES})


set(THROTTLE_TEST_SOURCE "${CMAKE_SOURCE_DIR}/planning/test/TestThrottle.c")
add_executable(ThrottleTest ${THROTTLE_TEST_SOURCE}
                ${RACING_SOURCES}
                ${PHYSICS_SOURCES}
            )
target_link_libraries(ThrottleTest Eigen3::Eigen m)

set(STEERING_TEST_SOURCE "${CMAKE_SOURCE_DIR}/planning/test/TestSteering.c")
add_executable(SteeringTest ${STEERING_TEST_SOURCE}
                ${RACING_SOURCES}
                ${PHYSICS_SOURCES}
            )
target_link_libraries(SteeringTest Eigen3::Eigen m)
                     