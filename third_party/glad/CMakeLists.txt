add_library(glad STATIC
  src/glad.c
)

target_include_directories(glad
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_compile_features(glad PUBLIC c_std_99)
