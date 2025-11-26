cmake_minimum_required(VERSION 3.20)
project(FSAI_Simulation_C LANGUAGES C CXX)

enable_testing()

# ------------------------------------------------------------------------------
# Standards
# ------------------------------------------------------------------------------
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Put all artifacts directly in the top-level build/ directory
# ------------------------------------------------------------------------------
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR})
# Handle multi-config generators (Xcode/VS)
if(CMAKE_CONFIGURATION_TYPES)
  foreach(cfg ${CMAKE_CONFIGURATION_TYPES})
    string(TOUPPER "${cfg}" CFGU)
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY_${CFGU} ${CMAKE_BINARY_DIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY_${CFGU} ${CMAKE_BINARY_DIR})
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY_${CFGU} ${CMAKE_BINARY_DIR})
  endforeach()
endif()

# ------------------------------------------------------------------------------
# Architecture / Homebrew / Paths
# ------------------------------------------------------------------------------
if(APPLE AND NOT DEFINED CMAKE_OSX_ARCHITECTURES)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "" FORCE)
    set(OpenCV_DIR /opt/homebrew/lib/cmake/opencv4)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "" FORCE)
    set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
  endif()
endif()

# ------------------------------------------------------------------------------
# ONNX Runtime smart discovery (version-aware)
# Priority:
#  1) onnxruntime_DIR / ONNXRuntime_DIR (explicit)
#  2) ONNXRUNTIME_ROOT (cache var or env) -> expects include/ and lib/
#  3) Newest under <project>/onnxruntime-osx-arm64-*  (unpacked tgz)
#  4) Newest under /opt/homebrew/onnxruntime/*        (manual “brew-like”)
# ------------------------------------------------------------------------------
set(ONNXRUNTIME_ROOT "" CACHE PATH "Path to ONNX Runtime root (with include/ and lib/)")
if(NOT ONNXRUNTIME_ROOT AND DEFINED ENV{ONNXRUNTIME_ROOT})
  set(ONNXRUNTIME_ROOT "$ENV{ONNXRUNTIME_ROOT}")
endif()

# If root is provided, derive package dir
if(NOT onnxruntime_DIR AND NOT ONNXRuntime_DIR AND ONNXRUNTIME_ROOT)
  if(EXISTS "${ONNXRUNTIME_ROOT}/lib/cmake/onnxruntime/onnxruntimeConfig.cmake")
    set(onnxruntime_DIR "${ONNXRUNTIME_ROOT}/lib/cmake/onnxruntime" CACHE PATH "" FORCE)
    list(APPEND CMAKE_PREFIX_PATH "${ONNXRUNTIME_ROOT}")
  endif()
endif()

# Collect candidate roots from project + homebrew paths
set(_ORT_CANDIDATES "")
if(NOT onnxruntime_DIR AND NOT ONNXRuntime_DIR)
  file(GLOB _proj_ort LIST_DIRECTORIES true
       "${CMAKE_SOURCE_DIR}/onnxruntime-osx-arm64-*")
  list(APPEND _ORT_CANDIDATES ${_proj_ort})

  if(APPLE)
    file(GLOB _brew_ort LIST_DIRECTORIES true
         "/opt/homebrew/onnxruntime/*")
    list(APPEND _ORT_CANDIDATES ${_brew_ort})
  endif()
endif()

# Pick newest by natural sort
if(_ORT_CANDIDATES AND NOT onnxruntime_DIR AND NOT ONNXRuntime_DIR AND NOT ONNXRUNTIME_ROOT)
  list(SORT _ORT_CANDIDATES COMPARE NATURAL)
  list(REVERSE _ORT_CANDIDATES)
  foreach(_cand ${_ORT_CANDIDATES})
    if(EXISTS "${_cand}/lib/cmake/onnxruntime/onnxruntimeConfig.cmake")
      set(onnxruntime_DIR "${_cand}/lib/cmake/onnxruntime" CACHE PATH "" FORCE)
      list(APPEND CMAKE_PREFIX_PATH "${_cand}")
      message(STATUS "Detected ONNX Runtime at: ${_cand}")
      break()
    endif()
  endforeach()
endif()


# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(SDL2 QUIET CONFIG)
find_package(Threads REQUIRED)

# onnx required by vision
find_package(onnxruntime QUIET CONFIG)
if(NOT onnxruntime_FOUND)
  find_package(ONNXRuntime QUIET CONFIG)
endif()
if(NOT (onnxruntime_FOUND OR ONNXRuntime_FOUND))
  message(FATAL_ERROR
    "onnxruntime not found.\n"
    "Set one of:\n"
    "  -Donnxruntime_DIR=/path/to/lib/cmake/onnxruntime\n"
    "  -DONNXRUNTIME_ROOT=/path/to/onnxruntime (with include/ and lib/)\n"
    "  or place a version under /opt/homebrew/onnxruntime/<version> on macOS."
  )
else()
  message(STATUS "ONNX Runtime located via CMake package config.")
endif()

set(SDL2_IMPORTED_TARGET "")
set(SDL2_INCLUDE_DIRS "")
if(TARGET SDL2::SDL2)
  set(SDL2_IMPORTED_TARGET SDL2::SDL2)
  get_target_property(SDL2_INCLUDE_DIRS SDL2::SDL2 INTERFACE_INCLUDE_DIRECTORIES)
elseif(TARGET SDL2::SDL2-static)
  set(SDL2_IMPORTED_TARGET SDL2::SDL2-static)
  get_target_property(SDL2_INCLUDE_DIRS SDL2::SDL2-static INTERFACE_INCLUDE_DIRECTORIES)
endif()

if(SDL2_IMPORTED_TARGET STREQUAL "")
  pkg_check_modules(SDL2 REQUIRED IMPORTED_TARGET sdl2)
  set(SDL2_IMPORTED_TARGET PkgConfig::SDL2)
  set(SDL2_INCLUDE_DIRS ${SDL2_INCLUDE_DIRS})
endif()
find_package(yaml-cpp REQUIRED)
find_package(OpenGL REQUIRED)
find_package(OpenCV REQUIRED)

# CGAL and its componentsx
find_package(CGAL QUIET COMPONENTS)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS)
if ( NOT CGAL_FOUND )
  message(STATUS "This project requires the CGAL library, and will not be compiled.")
  return()
endif()

find_package( Boost REQUIRED )
if ( NOT Boost_FOUND )
  message(STATUS "This project requires the Boost library, and will not be compiled.")
  return()
endif()

add_definitions(-DSDL_MAIN_HANDLED)

add_subdirectory(third_party/imgui)

# ------------------------------------------------------------------------------
# Collect sources/headers (exclude build tree)  [UNCHANGED]
# ------------------------------------------------------------------------------
file(GLOB_RECURSE ALL_C_SOURCES   CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.c")
file(GLOB_RECURSE ALL_CPP_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE ALL_HEADERS     CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.h" "${CMAKE_SOURCE_DIR}/*.hpp")

list(FILTER ALL_C_SOURCES   EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")
list(FILTER ALL_CPP_SOURCES EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")
list(FILTER ALL_HEADERS     EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")
list(FILTER ALL_C_SOURCES   EXCLUDE REGEX ".*/velox/.*")
list(FILTER ALL_CPP_SOURCES EXCLUDE REGEX ".*/velox/.*")
list(FILTER ALL_HEADERS     EXCLUDE REGEX ".*/velox/.*")

set(VELOX_HEADER_DIR "${CMAKE_SOURCE_DIR}/velox/lib")

# Build a list of header parent dirs so bare #include "X.h" works
set(HEADER_DIRS "")
foreach(h ${ALL_HEADERS})
  get_filename_component(hdir "${h}" DIRECTORY)
  if(NOT hdir MATCHES "^${CMAKE_BINARY_DIR}(/|$)")
    list(APPEND HEADER_DIRS "${hdir}")
  endif()
endforeach()
list(PREPEND HEADER_DIRS "${CMAKE_SOURCE_DIR}/common/include")
list(APPEND HEADER_DIRS "${CMAKE_SOURCE_DIR}/third_party/glad/include")
list(APPEND HEADER_DIRS
  "${CMAKE_SOURCE_DIR}/control/include"
  "${CMAKE_SOURCE_DIR}/io/include"
  "${CMAKE_SOURCE_DIR}/sim/include"
  "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/include"
  "${CMAKE_SOURCE_DIR}/vision_test/runtime_cpp/include"
  "${VELOX_HEADER_DIR}"
  "${VELOX_HEADER_DIR}/telemetry"
)
list(REMOVE_DUPLICATES HEADER_DIRS)

# Centralized telemetry implementation (console + structured)
add_library(fsai_telemetry STATIC
  ${CMAKE_SOURCE_DIR}/common/src/telemetry/Telemetry.cpp
  ${CMAKE_SOURCE_DIR}/velox/lib/telemetry/telemetry.cpp
)
target_include_directories(fsai_telemetry PUBLIC
  ${CMAKE_SOURCE_DIR}/common/include
  ${VELOX_HEADER_DIR}
  ${HEADER_DIRS}
)

# ------------------------------------------------------------------------------
# Vision-test libraries  [UNCHANGED]
# ------------------------------------------------------------------------------
add_subdirectory(vision_test/runtime_cpp/src)
target_include_directories(fsai_vision_runtime PUBLIC
    "${CMAKE_SOURCE_DIR}/common/include"
    "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/include"
    "${CMAKE_SOURCE_DIR}"

    # This path lets vision use the logger
    "${CMAKE_SOURCE_DIR}/sim/include/"
)
file(GLOB VISION_NODE_SOURCES
    "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/src/*.cpp"
    "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/providers/*.cpp"
)
target_sources(fsai_vision_runtime PRIVATE
    ${VISION_NODE_SOURCES}
)
target_include_directories(fsai_vision_runtime PUBLIC
    "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/include"
    "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/src"
    "${CMAKE_SOURCE_DIR}/vision_test/runtime_cpp/src"
    "${CMAKE_SOURCE_DIR}/sim/include"
    "${CMAKE_SOURCE_DIR}"
)
target_link_libraries(fsai_vision_runtime PUBLIC
    Eigen3::Eigen
    onnxruntime::onnxruntime
    ${OpenCV_LIBRARIES}
    Threads::Threads
    fsai_telemetry
)

# ------------------------------------------------------------------------------
# Simulation libraries  [UNCHANGED]
# ------------------------------------------------------------------------------
set(SIM_C_SOURCES   ${ALL_C_SOURCES})
set(SIM_CPP_SOURCES ${ALL_CPP_SOURCES})
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/(main|sim/app)/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/(main|sim/app)/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/third_party/imgui/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/third_party/imgui/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/(test|tests)/.*|.*/.*Test.*\\.(c)$")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/(test|tests)/.*|.*/.*Test.*\\.(cpp)$")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/tools/Graphics/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/tools/Graphics/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/sim/svcu/svcu_main.cpp$")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/.git/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/.git/.*")

# Drop everything under vision/
# Re-introduce vision
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/vision/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/vision/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/vision_test/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/vision_test/.*")
# Use the velox telemetry implementation via the dedicated library
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/common/src/telemetry/telemetry.cpp$")

list(APPEND SIM_C_SOURCES "${CMAKE_SOURCE_DIR}/control/controller.prot.c")
file(GLOB CONTROL_SOURCE "${CMAKE_SOURCE_DIR}/control/fsai_plan.cpp")

set(VELOX_PARAM_ROOT_DIR "${CMAKE_SOURCE_DIR}/velox/parameters")

add_library(fsai_sim STATIC
  ${SIM_C_SOURCES}
  ${SIM_CPP_SOURCES}
  ${ALL_HEADERS}
)
target_include_directories(fsai_sim PUBLIC ${HEADER_DIRS})
target_link_libraries(fsai_sim PUBLIC
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  OpenGL::GL
  Threads::Threads
  CGAL::CGAL
  m
  onnxruntime::onnxruntime
  ${OpenCV_LIBRARIES}
  fsai_telemetry
)
target_compile_definitions(fsai_sim PUBLIC VELOX_PARAM_ROOT="${VELOX_PARAM_ROOT_DIR}")


add_library(fsai_ctrl STATIC
  ${SIM_C_SOURCES}
  ${SIM_CPP_SOURCES}
  ${ALL_HEADERS}
)
target_include_directories(fsai_ctrl PUBLIC ${HEADER_DIRS})
target_link_libraries(fsai_ctrl PUBLIC
  Eigen3::Eigen
  CGAL::CGAL
  m
  yaml-cpp::yaml-cpp
  fsai_telemetry
)
target_compile_definitions(fsai_ctrl PUBLIC VELOX_PARAM_ROOT="${VELOX_PARAM_ROOT_DIR}")

file(GLOB SIM_UNIT_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/sim/src/tests/*.cpp")
if(SIM_UNIT_TEST_SOURCES)
  add_executable(sim_unit_tests ${SIM_UNIT_TEST_SOURCES})
  target_link_libraries(sim_unit_tests PRIVATE fsai_sim Threads::Threads)
  target_compile_definitions(sim_unit_tests PRIVATE FSAI_PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
  add_test(NAME sim_unit_tests COMMAND sim_unit_tests)
endif()

if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_sim  PUBLIC ${SDL2_IMPORTED_TARGET})
  target_link_libraries(fsai_ctrl PUBLIC ${SDL2_IMPORTED_TARGET})
endif()
if(SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_sim  PUBLIC ${SDL2_INCLUDE_DIRS})
  target_include_directories(fsai_ctrl PUBLIC ${SDL2_INCLUDE_DIRS})
endif()

file(GLOB SVCU_TEST_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/sim/svcu/tests/*.cpp")
if(SVCU_TEST_SOURCES)
  add_executable(svcu_integration_tests ${SVCU_TEST_SOURCES})
  target_link_libraries(svcu_integration_tests PRIVATE fsai_sim Threads::Threads)
  if(TARGET fsai_control_runtime)
    target_link_libraries(svcu_integration_tests PRIVATE fsai_control_runtime)
  endif()
  target_compile_definitions(svcu_integration_tests PRIVATE FSAI_PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
  add_test(NAME svcu_integration_tests COMMAND svcu_integration_tests)
endif()

file(GLOB RENDERING_C_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/tools/Graphics/*.c")
add_library(fsai_rendering STATIC ${RENDERING_C_SOURCES})
target_include_directories(fsai_rendering PUBLIC ${HEADER_DIRS})
if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_rendering PUBLIC ${SDL2_IMPORTED_TARGET})
endif()
if(SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_rendering PUBLIC ${SDL2_INCLUDE_DIRS})
endif()

# ------------------------------------------------------------------------------
# Simulation executable  [UNCHANGED]
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SIM_APP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/sim/app/*.c"
  "${CMAKE_SOURCE_DIR}/sim/app/*.cpp"
)
add_executable(fsai_run ${SIM_APP_SOURCES})
target_include_directories(fsai_run PRIVATE
    "${CMAKE_SOURCE_DIR}/common/include"
    "${CMAKE_SOURCE_DIR}/vision/runtime_cpp/include"
    "${CMAKE_SOURCE_DIR}"
    "${CMAKE_SOURCE_DIR}/vision_test/runtime_cpp/src"
    "${CMAKE_SOURCE_DIR}/vision_test/runtime_cpp/include"
    "${CMAKE_SOURCE_DIR}/sim/include"
)
target_compile_definitions(fsai_run PRIVATE FSAI_PROJECT_ROOT="${CMAKE_SOURCE_DIR}")
target_link_libraries(fsai_run PRIVATE fsai_sim fsai_rendering imgui fsai_telemetry)
if(TARGET fsai_vision_runtime)
  target_link_libraries(fsai_run PRIVATE fsai_vision_runtime)
endif()

file(GLOB_RECURSE CONTROL_RUNTIME_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/control/runtime_cpp/*.cpp"
)
if(CONTROL_RUNTIME_SOURCES)
  add_library(fsai_control_runtime STATIC ${CONTROL_RUNTIME_SOURCES})
  target_include_directories(fsai_control_runtime PUBLIC ${HEADER_DIRS})
  target_link_libraries(fsai_control_runtime PUBLIC fsai_sim)
  if(UNIX)
    target_link_libraries(fsai_control_runtime PUBLIC dl)
  endif()
  target_link_libraries(fsai_run PRIVATE fsai_control_runtime)
  if(TARGET svcu_integration_tests)
    target_link_libraries(svcu_integration_tests PRIVATE fsai_control_runtime)
  endif()
endif()

add_executable(svcu_run sim/svcu/svcu_main.cpp)
target_link_libraries(svcu_run PRIVATE fsai_sim)

# ------------------------------------------------------------------------------
# Control prototype  [UNCHANGED]
# ------------------------------------------------------------------------------
add_executable(fsai_plan ${CONTROL_SOURCE})
target_link_libraries(fsai_plan PRIVATE fsai_ctrl imgui)
add_to_cached_list( CGAL_EXECUTABLE_TARGETS fsai_plan )
if(TARGET SDL2::SDL2)
  target_link_libraries(fsai_plan PRIVATE SDL2::SDL2)
else()
  target_link_libraries(fsai_plan PRIVATE PkgConfig::SDL2)
endif()

# ------------------------------------------------------------------------------
# Tests  [UNCHANGED]
# ------------------------------------------------------------------------------
option(BUILD_TESTS "Build test executables automatically" ON)
if(BUILD_TESTS)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/*Test*.c"
    "${CMAKE_SOURCE_DIR}/*Test*.cpp"
  )
  list(FILTER TEST_SOURCES EXCLUDE REGEX ".*/vision_test/.*")
  foreach(TEST_FILE ${TEST_SOURCES})
    if(TEST_FILE MATCHES "/sim/svcu/tests/")
      continue()
    endif()
    get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
    if(TARGET ${TEST_NAME})
      continue()
    endif()
    add_executable(${TEST_NAME} ${TEST_FILE})
    target_link_libraries(${TEST_NAME} PRIVATE fsai_sim fsai_rendering)
    if(SDL2_IMPORTED_TARGET)
      target_link_libraries(${TEST_NAME} PRIVATE ${SDL2_IMPORTED_TARGET})
    endif()
    message(STATUS "Added test: ${TEST_NAME}")
  endforeach()
endif()

# ------------------------------------------------------------------------------
# Make "build everything" the default when you run `cmake --build build`
# ------------------------------------------------------------------------------
# Gather all known build targets in this directory and depend on them.
# This preserves your globs and avoids naming explicit targets.
get_property(_ALL_TARGETS DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY BUILDSYSTEM_TARGETS)
# Avoid self-dependency if reconfiguring
list(REMOVE_ITEM _ALL_TARGETS build_all)
if(_ALL_TARGETS)
  add_custom_target(build_all ALL DEPENDS ${_ALL_TARGETS})
else()
  message(WARNING "No build targets were created — check that your globs pick up sources.")
endif()

# ------------------------------------------------------------------------------
# Diagnostics
# ------------------------------------------------------------------------------
message(STATUS "CMake Generator: ${CMAKE_GENERATOR}")
message(STATUS "Architecture: ${CMAKE_OSX_ARCHITECTURES}")
message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Header include dirs count: ${HEADER_DIRS}")
