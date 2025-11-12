cmake_minimum_required(VERSION 3.28.3)

project(test_feature_matching) 

find_package(OpenCV REQUIRED)
find_package(ONNXRuntime REQUIRED) 

add_executable(${PROJECT_NAME} features.cpp)

target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ONNXRuntime::ONNXRuntime)
