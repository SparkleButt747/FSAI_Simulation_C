cmake_minimum_required(VERSION 3.20)
project(FSAI_Simulation_C LANGUAGES C CXX)

# ------------------------------------------------------------------------------
# Standards
# ------------------------------------------------------------------------------
set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ------------------------------------------------------------------------------
# Architecture / Homebrew / Paths
# ------------------------------------------------------------------------------
if(APPLE AND NOT DEFINED CMAKE_OSX_ARCHITECTURES)
  if(CMAKE_SYSTEM_PROCESSOR MATCHES "arm64|aarch64")
    set(CMAKE_OSX_ARCHITECTURES "arm64" CACHE STRING "" FORCE)
  elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "x86_64")
    set(CMAKE_OSX_ARCHITECTURES "x86_64" CACHE STRING "" FORCE)
  endif()
endif()

if(APPLE)
  find_program(BREW brew PATHS /opt/homebrew/bin /usr/local/bin NO_DEFAULT_PATH)
  if(BREW)
    execute_process(COMMAND "${BREW}" --prefix
                    OUTPUT_VARIABLE HOMEBREW_PREFIX
                    OUTPUT_STRIP_TRAILING_WHITESPACE)
    if(EXISTS "${HOMEBREW_PREFIX}")
      list(APPEND CMAKE_PREFIX_PATH "${HOMEBREW_PREFIX}")
      set(ENV{PKG_CONFIG_PATH}
          "${HOMEBREW_PREFIX}/lib/pkgconfig:${HOMEBREW_PREFIX}/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
    endif()
  endif()
endif()

# ------------------------------------------------------------------------------
# Dependencies
# ------------------------------------------------------------------------------
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(PkgConfig REQUIRED)
find_package(SDL2 QUIET CONFIG)
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

add_definitions(-DSDL_MAIN_HANDLED)

# ------------------------------------------------------------------------------
# Collect sources/headers (exclude build tree)
# ------------------------------------------------------------------------------
file(GLOB_RECURSE ALL_C_SOURCES   CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.c")
file(GLOB_RECURSE ALL_CPP_SOURCES CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.cpp")
file(GLOB_RECURSE ALL_HEADERS     CONFIGURE_DEPENDS "${CMAKE_SOURCE_DIR}/*.h" "${CMAKE_SOURCE_DIR}/*.hpp")

list(FILTER ALL_C_SOURCES   EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")
list(FILTER ALL_CPP_SOURCES EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")
list(FILTER ALL_HEADERS     EXCLUDE REGEX "^${CMAKE_BINARY_DIR}/")

# Build a list of header parent dirs so bare #include "X.h" works
set(HEADER_DIRS "")
foreach(h ${ALL_HEADERS})
  get_filename_component(hdir "${h}" DIRECTORY)
  if(NOT hdir MATCHES "^${CMAKE_BINARY_DIR}(/|$)")
    list(APPEND HEADER_DIRS "${hdir}")
  endif()
endforeach()
list(APPEND HEADER_DIRS "${CMAKE_SOURCE_DIR}/third_party/glad/include")
list(REMOVE_DUPLICATES HEADER_DIRS)

# ------------------------------------------------------------------------------
# Simulation libraries
# ------------------------------------------------------------------------------
set(SIM_C_SOURCES   ${ALL_C_SOURCES})
set(SIM_CPP_SOURCES ${ALL_CPP_SOURCES})

list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/(main|sim/app)/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/(main|sim/app)/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/(test|tests)/.*|.*/.*Test.*\\.(c)$")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/(test|tests)/.*|.*/.*Test.*\\.(cpp)$")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/control/BoundaryEstimation.cpp$")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/control/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/control/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/tools/Graphics/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/tools/Graphics/.*")

list(APPEND SIM_C_SOURCES "${CMAKE_SOURCE_DIR}/control/RacingAlgorithm.c")

add_library(fsai_sim STATIC
  ${SIM_C_SOURCES}
  ${SIM_CPP_SOURCES}
  ${ALL_HEADERS}         # for IDEs
)

target_include_directories(fsai_sim
  PUBLIC
    ${HEADER_DIRS}
)

target_link_libraries(fsai_sim
  PUBLIC
    Eigen3::Eigen
    yaml-cpp::yaml-cpp
    OpenGL::GL
    m
)

if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_sim PUBLIC ${SDL2_IMPORTED_TARGET})
endif()

if(SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_sim PUBLIC ${SDL2_INCLUDE_DIRS})
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
# Simulation executable
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SIM_APP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/sim/app/*.c"
  "${CMAKE_SOURCE_DIR}/sim/app/*.cpp"
)

add_executable(fsai_run ${SIM_APP_SOURCES})
target_link_libraries(fsai_run PRIVATE fsai_sim fsai_rendering)

# ------------------------------------------------------------------------------
# Tests
# ------------------------------------------------------------------------------
option(BUILD_TESTS "Build test executables automatically" ON)
if(BUILD_TESTS)
  file(GLOB_RECURSE TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_SOURCE_DIR}/*Test*.c"
    "${CMAKE_SOURCE_DIR}/*Test*.cpp"
  )
    foreach(TEST_FILE ${TEST_SOURCES})
      get_filename_component(TEST_NAME ${TEST_FILE} NAME_WE)
      add_executable(${TEST_NAME} ${TEST_FILE})
      target_link_libraries(${TEST_NAME} PRIVATE fsai_sim fsai_rendering)
      if(SDL2_IMPORTED_TARGET)
        target_link_libraries(${TEST_NAME} PRIVATE ${SDL2_IMPORTED_TARGET})
      endif()
      message(STATUS "Added test: ${TEST_NAME}")
    endforeach()
endif()

# ------------------------------------------------------------------------------
# Diagnostics
# ------------------------------------------------------------------------------
message(STATUS "CMake Generator: ${CMAKE_GENERATOR}")
message(STATUS "Architecture: ${CMAKE_OSX_ARCHITECTURES}")
message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Header include dirs count: ${HEADER_DIRS}")
