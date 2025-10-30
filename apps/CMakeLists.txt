file(GLOB APP_SUPPORT_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(fsai_app_support STATIC ${APP_SUPPORT_SOURCES})

target_include_directories(fsai_app_support
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fsai_app_support
  PUBLIC
    fsai_common
)

if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_app_support PUBLIC ${SDL2_IMPORTED_TARGET})
endif()

add_executable(fsai_run
  ${CMAKE_CURRENT_SOURCE_DIR}/fsai_run.cpp
)

target_include_directories(fsai_run PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/include)

target_link_libraries(fsai_run
  PRIVATE
    fsai_sim
    fsai_control
    fsai_io
    fsai_app_support
    imgui
)

add_executable(svcu_run
  ${CMAKE_CURRENT_SOURCE_DIR}/svcu_run.cpp
)

target_link_libraries(svcu_run
  PRIVATE
    fsai_sim
    fsai_io
    fsai_app_support
)
