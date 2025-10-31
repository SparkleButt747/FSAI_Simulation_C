cmake_minimum_required(VERSION 3.15)

project(YOLO_DETECTION)

set(ORT_ROOT "/usr/homebrew/onnxruntime/")

find_path(ORT_INCLUDE_DIR onnxruntime_cxx_api.h
    HINTS ${ORT_ROOT}/include
)

find_library(ORT_LIBRARY onnxruntime
    HINTS ${ORT_ROOT}/lib
)

find_package(OpenCV REQUIRED)

find_package(CUDAToolkit REQUIRED)

add_executable(detector detect.cpp)

target_include_directories(detector PUBLIC
    ${ORT_INCLUDE_DIR}
    ${OpenCv_INCLUDE_DIRS}
)

target_link_libraries(detector PUBLIC
    ${ORT_LIBRARY}
    ${OpenCV_LIBS}
    cudart  # This links the CUDA runtime
)

if(ORT_INCLUDE_DIR AND ORT_LIBRARY)
    message(STATUS "Found ONNX Runtime: ${ORT_LIBRARY}")
    message(STATUS "Found ONNX Includes: ${ORT_INCLUDE_DIR}")
else()
    message(FATAL_ERROR "Could not find ONNX Runtime. Check ORT_ROOT path.")
endif() 