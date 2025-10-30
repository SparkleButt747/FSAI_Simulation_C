file(GLOB IO_CAMERA_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/camera/sim_stereo/*.cpp")

set(IO_CAN_SOURCES
  ${CMAKE_CURRENT_SOURCE_DIR}/can/src/ai2vcu_adapter.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/can/src/can_iface.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/can/src/can_link.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/can/src/udp_can.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/can/src/loopback_can.cpp)

if(UNIX AND NOT APPLE)
  list(APPEND IO_CAN_SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/can/src/socketcan.cpp)
  set(_fsai_can_backend FSAI_CAN_SOCKETCAN)
else()
  set(_fsai_can_backend FSAI_CAN_UDP)
endif()

add_library(fsai_io STATIC
  ${IO_CAMERA_SOURCES}
  ${IO_CAN_SOURCES}
  ${PROJECT_SOURCE_DIR}/generated/can/adsdv_dbc.cpp)

if(FSAI_SDL2_INCLUDE_DIRS)
  target_include_directories(fsai_io
    PUBLIC
      ${FSAI_SDL2_INCLUDE_DIRS})
endif()

target_include_directories(fsai_io
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/camera
    ${CMAKE_CURRENT_SOURCE_DIR}/camera/sim_stereo
    ${CMAKE_CURRENT_SOURCE_DIR}/can/include
    ${CMAKE_CURRENT_SOURCE_DIR}/can/include/io/can
    ${PROJECT_SOURCE_DIR}/generated/can)

target_link_libraries(fsai_io
  PUBLIC
    fsai_common
    glad
    Threads::Threads)

if(FSAI_SDL2_TARGET)
  target_link_libraries(fsai_io PUBLIC ${FSAI_SDL2_TARGET})
endif()

if(OpenGL_FOUND)
  target_link_libraries(fsai_io PUBLIC OpenGL::GL)
endif()

if(UNIX)
  target_link_libraries(fsai_io PUBLIC dl)
endif()

target_compile_definitions(fsai_io PUBLIC ${_fsai_can_backend}=1)
