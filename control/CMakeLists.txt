file(GLOB FSAI_CONTROL_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp"
)

list(FILTER FSAI_CONTROL_SOURCES EXCLUDE REGEX "centerline\\.prot\\.cpp$")

add_library(fsai_control STATIC
  ${FSAI_CONTROL_SOURCES}
)

target_include_directories(fsai_control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

target_link_libraries(fsai_control
  PUBLIC
    fsai_common
    fsai_io
    Eigen3::Eigen
    CGAL::CGAL
)

target_compile_features(fsai_control PUBLIC c_std_99 cxx_std_20)

if(FSAI_ENABLE_TESTS)
  file(GLOB FSAI_CONTROL_TESTS CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/tests/*.cpp"
  )
  foreach(test_src ${FSAI_CONTROL_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE fsai_control m)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()

add_executable(fsai_plan
  ${CMAKE_CURRENT_SOURCE_DIR}/src/centerline.prot.cpp
)

target_link_libraries(fsai_plan PRIVATE fsai_control)
if(SDL2_IMPORTED_TARGET)
  target_link_libraries(fsai_plan PRIVATE ${SDL2_IMPORTED_TARGET})
endif()
add_to_cached_list(CGAL_EXECUTABLE_TARGETS fsai_plan)
