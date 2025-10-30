file(GLOB_RECURSE FSAI_SIM_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c")
file(GLOB_RECURSE FSAI_SIM_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.hpp"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.h")
list(FILTER FSAI_SIM_SOURCES EXCLUDE REGEX "/tests/")

add_library(fsai_sim STATIC ${FSAI_SIM_SOURCES})

if(FSAI_SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_sim
    PUBLIC
      ${FSAI_SDL2_INCLUDE_DIRS})
endif()

set(_fsai_sim_include_dirs
  ${CMAKE_CURRENT_SOURCE_DIR}/src
  ${PROJECT_SOURCE_DIR}/generated/can
  ${PROJECT_SOURCE_DIR}/io/can/include
  ${PROJECT_SOURCE_DIR}/io/can/include/io/can)
foreach(header ${FSAI_SIM_HEADERS})
  get_filename_component(dir "${header}" DIRECTORY)
  list(APPEND _fsai_sim_include_dirs "${dir}")
endforeach()
list(REMOVE_DUPLICATES _fsai_sim_include_dirs)

target_include_directories(fsai_sim
  PUBLIC
    ${_fsai_sim_include_dirs})

target_link_libraries(fsai_sim
  PUBLIC
    fsai_common
    fsai_io
    Eigen3::Eigen
    yaml-cpp
    ${FSAI_SDL2_TARGET}
    Threads::Threads)

if(BUILD_TESTING)
  file(GLOB FSAI_SVCU_TESTS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/src/svcu/tests/*.cpp")
  if(FSAI_SVCU_TESTS)
    add_executable(svcu_integration_tests ${FSAI_SVCU_TESTS})
    target_link_libraries(svcu_integration_tests PRIVATE fsai_sim)
    target_compile_definitions(svcu_integration_tests PRIVATE FSAI_PROJECT_ROOT="${PROJECT_SOURCE_DIR}")
    add_test(NAME svcu_integration_tests COMMAND svcu_integration_tests)
  endif()
endif()
