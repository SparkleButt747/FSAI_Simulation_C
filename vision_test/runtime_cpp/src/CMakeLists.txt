set(VISION_RUNTIME_SOURCES
  detect.cpp
  frame_ring_buffer.cpp
)

add_library(fsai_vision_runtime STATIC ${VISION_RUNTIME_SOURCES})

target_include_directories(fsai_vision_runtime
  PUBLIC
    ${HEADER_DIRS}
)

