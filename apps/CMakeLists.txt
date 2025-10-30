if(NOT FSAI_BUILD_APPS)
  return()
endif()

add_executable(fsai_run
  ${CMAKE_CURRENT_SOURCE_DIR}/sim_viewer/fsai_run.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/sim_viewer/graphics/Graphics.c)

target_include_directories(fsai_run
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/sim_viewer
    ${CMAKE_CURRENT_SOURCE_DIR}/sim_viewer/graphics)
if(FSAI_SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_run PRIVATE ${FSAI_SDL2_INCLUDE_DIRS})
endif()

target_link_libraries(fsai_run
  PRIVATE
    fsai_common
    fsai_io
    fsai_sim
    fsai_control
    yaml-cpp
    imgui
    glad
    Threads::Threads)

if(FSAI_SDL2_TARGET)
  target_link_libraries(fsai_run PRIVATE ${FSAI_SDL2_TARGET})
endif()

if(OpenGL_FOUND)
  target_link_libraries(fsai_run PRIVATE OpenGL::GL)
endif()

add_executable(svcu_run ${CMAKE_CURRENT_SOURCE_DIR}/svcu_runner/main.cpp)

target_include_directories(svcu_run
  PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/svcu_runner)

target_link_libraries(svcu_run
  PRIVATE
    fsai_common
    fsai_io
    fsai_sim
    Threads::Threads)
