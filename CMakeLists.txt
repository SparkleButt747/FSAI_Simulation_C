cmake_minimum_required(VERSION 3.10)

# Project name and language.
project(FSAI_Simulation C)

# Use C99 standard.
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)

# Include directories for headers.
include_directories(${CMAKE_SOURCE_DIR}/MainController)
include_directories(${CMAKE_SOURCE_DIR}/PathLogic)
include_directories(${CMAKE_SOURCE_DIR}/PhysicsEngine)
include_directories(${CMAKE_SOURCE_DIR}/RacingAlgorithms)
include_directories(${CMAKE_SOURCE_DIR}/SimulationModels/DynamicBicycle)
include_directories(${CMAKE_SOURCE_DIR}/Telemetry)
include_directories(${CMAKE_SOURCE_DIR}/Graphics)

# Add system include directory for libyaml.
include_directories(/usr/local/include)
link_directories(/usr/local/lib)
# If you're on Apple Silicon, you might need:
# include_directories(/opt/homebrew/include)
# link_directories(/opt/homebrew/lib)

# Find SDL2
find_package(SDL2 REQUIRED)
include_directories(${SDL2_INCLUDE_DIRS})

# ------------------------------------------------------------------------------
# Build the SimulationModels test executable.
# ------------------------------------------------------------------------------

# Gather all C source files from the DynamicBicycle folder.
file(GLOB DYNAMIC_BICYCLE_SOURCES
    "${CMAKE_SOURCE_DIR}/SimulationModels/DynamicBicycle/*.c"
)
file(GLOB KEY_SOURCES "${CMAKE_SOURCE_DIR}/MainController/KeyboardInputHandler.c")
file(GLOB GRAPHICS_SOURCES "${CMAKE_SOURCE_DIR}/Graphics/*.c")
file(GLOB GRAPHICS_H_SOURCES "${CMAKE_SOURCE_DIR}/Graphics/*.h")

# The test driver for SimulationModels is at SimulationModels/test.c.
set(SIMULATION_TEST_SOURCE "${CMAKE_SOURCE_DIR}/SimulationModels/test.c")

# Create the SimulationModels executable.
add_executable(SimulationModelsTest ${SIMULATION_TEST_SOURCE} ${DYNAMIC_BICYCLE_SOURCES} ${KEY_SOURCES} ${GRAPHICS_SOURCES} ${GRAPHICS_H_SOURCES})

# Link the libyaml library. You can use either option 1 or 2. option 1 is works for me.
# Option 1: Link using full path
# target_link_libraries(SimulationModelsTest /usr/local/lib/libyaml.dylib)
target_link_libraries(SimulationModelsTest /usr/local/lib/libyaml.dylib ${SDL2_LIBRARIES})

# Option 2: Or use find_library
# find_library(YAML_LIB NAMES yaml PATHS /usr/local/lib)
# if(YAML_LIB)
#     target_link_libraries(SimulationModelsTest ${YAML_LIB})
# else()
#     message(FATAL_ERROR "libyaml not found")
# endif()

# ------------------------------------------------------------------------------
# Build the PathLogic test executable.
# ------------------------------------------------------------------------------

# Gather all C source files from the PathLogic folder.
file(GLOB PATH_LOGIC_SOURCES
    "${CMAKE_SOURCE_DIR}/PathLogic/*.c"
)

# The test driver for PathLogic is at PathLogic/test.c.
set(PATH_LOGIC_TEST_SOURCE "${CMAKE_SOURCE_DIR}/PathLogic/test.c")

# Create the PathLogic executable.
add_executable(PathLogicTest ${PATH_LOGIC_TEST_SOURCE} ${PATH_LOGIC_SOURCES})

# ------------------------------------------------------------------------------
# Build the Main simulation executable.
# ------------------------------------------------------------------------------

# Collect source files.
file(GLOB MAINCONTROLLER_SOURCES "${CMAKE_SOURCE_DIR}/MainController/*.c")

#Remove test.c from the list of source files
file(GLOB PATHLOGIC_SOURCES "${CMAKE_SOURCE_DIR}/PathLogic/*.c")
list(REMOVE_ITEM PATHLOGIC_SOURCES "${CMAKE_SOURCE_DIR}/PathLogic/test.c")

file(GLOB PHYSICS_SOURCES "${CMAKE_SOURCE_DIR}/PhysicsEngine/*.c")
file(GLOB RACING_SOURCES "${CMAKE_SOURCE_DIR}/RacingAlgorithms/*.c")

#Remove test.c from the list of source files
file(GLOB SIMULATIONMODELS_SOURCES "${CMAKE_SOURCE_DIR}/SimulationModels/DynamicBicycle/*.c")
list(REMOVE_ITEM SIMULATIONMODELS_SOURCES "${CMAKE_SOURCE_DIR}/SimulationModels/DynamicBicycle/test.c")

file(GLOB TELEMETRY_SOURCES "${CMAKE_SOURCE_DIR}/Telemetry/*.c")

set(SIMULATION_SOURCE_FINAL "${CMAKE_SOURCE_DIR}/MainController/main.c")

add_executable(MainController
    ${SIMULATION_SOURCE_FINAL}
    ${MAINCONTROLLER_SOURCES}
    ${PATHLOGIC_SOURCES}
    ${PHYSICS_SOURCES}
    ${RACING_SOURCES}
    ${SIMULATIONMODELS_SOURCES}
    ${TELEMETRY_SOURCES}
    ${KEY_SOURCES}
    ${GRAPHICS_SOURCES}
    ${GRAPHICS_H_SOURCES}
)

target_link_libraries(MainController /usr/local/lib/libyaml.dylib ${SDL2_LIBRARIES})

