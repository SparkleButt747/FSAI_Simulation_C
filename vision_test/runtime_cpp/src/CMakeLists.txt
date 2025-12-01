set(VISION_RUNTIME_SOURCES
  edge_preview.cpp
        detect.cpp
  frame_ring_buffer.cpp
)

add_library(fsai_vision_runtime STATIC ${VISION_RUNTIME_SOURCES})

target_include_directories(fsai_vision_runtime
  PUBLIC
    ${CMAKE_SOURCE_DIR}/common/include
    ${CMAKE_SOURCE_DIR}/vision_test/runtime_cpp/include
    ${CMAKE_SOURCE_DIR}/common/include/common
    ${OpenCV_INCLUDE_DIRS}
    ${SDL2_INCLUDE_DIRS}
)

target_link_libraries(fsai_vision_runtime
  PUBLIC
    ${OpenCV_LIBS}
    ${SDL2_IMPORTED_TARGET}
)

