file(GLOB_RECURSE FSAI_SIM_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

add_library(fsai_sim STATIC
  ${FSAI_SIM_SOURCES}
)

target_include_directories(fsai_sim
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${PROJECT_SOURCE_DIR}/control/include
)

target_link_libraries(fsai_sim
  PUBLIC
    fsai_common
    fsai_io
    yaml-cpp::yaml-cpp
    Eigen3::Eigen
    Threads::Threads
    OpenGL::GL
)

if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_sim PUBLIC ${SDL2_IMPORTED_TARGET})
endif()

target_compile_features(fsai_sim PUBLIC cxx_std_20)

if(FSAI_ENABLE_TESTS)
  file(GLOB_RECURSE FSAI_SIM_TEST_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.c"
  )
  foreach(test_src ${FSAI_SIM_TEST_SOURCES})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE fsai_sim fsai_io fsai_common)
    if(SDL2_IMPORTED_TARGET)
      target_link_libraries(${test_name} PRIVATE ${SDL2_IMPORTED_TARGET})
    endif()
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
