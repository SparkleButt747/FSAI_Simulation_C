file(GLOB_RECURSE FSAI_COMMON_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(fsai_common STATIC
  ${FSAI_COMMON_SOURCES}
)

target_include_directories(fsai_common
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src/ui
)

target_link_libraries(fsai_common
  PUBLIC
    m
)

target_compile_features(fsai_common PUBLIC c_std_99 cxx_std_20)
