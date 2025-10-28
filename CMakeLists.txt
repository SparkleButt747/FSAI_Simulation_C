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
find_package(Threads REQUIRED)
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

# CGAL and its components
find_package(CGAL QUIET COMPONENTS)
find_package(CGAL REQUIRED OPTIONAL_COMPONENTS Qt6)


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
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/third_party/imgui/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/third_party/imgui/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/(test|tests)/.*|.*/.*Test.*\\.(c)$")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/(test|tests)/.*|.*/.*Test.*\\.(cpp)$")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/control/centerline.prot.cpp$")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/control/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/control/.*")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/tools/Graphics/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/tools/Graphics/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/sim/svcu/svcu_main.cpp$")
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/.git/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/.git/.*")


# Drop everything under vision/
list(FILTER SIM_C_SOURCES   EXCLUDE REGEX ".*/vision/.*")
list(FILTER SIM_CPP_SOURCES EXCLUDE REGEX ".*/vision/.*")


list(APPEND SIM_C_SOURCES "${CMAKE_SOURCE_DIR}/control/controller.prot.c")

# control prototype source
file(GLOB CONTROL_SOURCE "${CMAKE_SOURCE_DIR}/control/centerline.prot.cpp")

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
    Threads::Threads
    m
)


add_library(fsai_ctrl STATIC
  ${SIM_C_SOURCES}
  ${SIM_CPP_SOURCES}
  ${ALL_HEADERS}         # for IDEs
)

target_include_directories(fsai_ctrl
  PUBLIC
    ${HEADER_DIRS}
)

target_link_libraries(fsai_ctrl
  PUBLIC
    Eigen3::Eigen
    CGAL::CGAL
    CGAL::CGAL_Basic_viewer
    m
)

if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_sim PUBLIC ${SDL2_IMPORTED_TARGET})
  target_link_libraries(fsai_ctrl PUBLIC ${SDL2_IMPORTED_TARGET})
endif()

if(SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_sim PUBLIC ${SDL2_INCLUDE_DIRS})
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
# Simulation executable
# ------------------------------------------------------------------------------
file(GLOB_RECURSE SIM_APP_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_SOURCE_DIR}/sim/app/*.c"
  "${CMAKE_SOURCE_DIR}/sim/app/*.cpp"
)

add_executable(fsai_run ${SIM_APP_SOURCES})
target_link_libraries(fsai_run PRIVATE fsai_sim fsai_rendering imgui)

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
# Control prototype
# ------------------------------------------------------------------------------
add_executable(fsai_plan ${CONTROL_SOURCE})
target_link_libraries(fsai_plan PRIVATE fsai_ctrl)

add_to_cached_list( CGAL_EXECUTABLE_TARGETS fsai_plan )


if(TARGET SDL2::SDL2)
  target_link_libraries(fsai_plan PRIVATE SDL2::SDL2)
else()
  target_link_libraries(fsai_plan PRIVATE PkgConfig::SDL2)
endif()

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
# Diagnostics
# ------------------------------------------------------------------------------
message(STATUS "CMake Generator: ${CMAKE_GENERATOR}")
message(STATUS "Architecture: ${CMAKE_OSX_ARCHITECTURES}")
message(STATUS "Eigen include: ${EIGEN3_INCLUDE_DIRS}")
message(STATUS "Header include dirs count: ${HEADER_DIRS}")
