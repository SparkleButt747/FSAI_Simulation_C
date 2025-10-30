file(GLOB_RECURSE FSAI_IO_CAMERA_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/camera/**/*.cpp"
)

file(GLOB_RECURSE FSAI_IO_CAN_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/can/*.cpp"
)

add_library(fsai_io STATIC
  ${FSAI_IO_CAMERA_SOURCES}
  ${FSAI_IO_CAN_SOURCES}
)

target_include_directories(fsai_io
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/camera
    ${CMAKE_CURRENT_SOURCE_DIR}/can/include
)

target_link_libraries(fsai_io
  PUBLIC
    fsai_common
    glad
    Eigen3::Eigen
    OpenGL::GL
    Threads::Threads
)

if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_io PUBLIC ${SDL2_IMPORTED_TARGET})
endif()

target_compile_features(fsai_io PUBLIC cxx_std_20)

target_compile_definitions(fsai_io
  PRIVATE
    $<$<BOOL:${APPLE}>:FSAI_CAN_PLATFORM_UDP>
)

if(UNIX AND NOT APPLE)
  target_link_libraries(fsai_io PUBLIC dl)
endif()
