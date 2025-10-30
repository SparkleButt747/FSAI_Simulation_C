if(NOT FSAI_HAVE_CGAL)
  add_library(fsai_control INTERFACE)
  target_link_libraries(fsai_control INTERFACE fsai_common)
  message(WARNING "fsai_control built without CGAL; advanced planning features disabled")
  return()
endif()

file(GLOB FSAI_CONTROL_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")

add_library(fsai_control STATIC ${FSAI_CONTROL_SOURCES})

target_include_directories(fsai_control
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
    ${CMAKE_CURRENT_SOURCE_DIR}/src)

target_link_libraries(fsai_control
  PUBLIC
    fsai_common
    Eigen3::Eigen
    CGAL::CGAL
    CGAL::CGAL_Basic_viewer)

if(NOT WIN32)
  target_link_libraries(fsai_control PUBLIC m)
endif()

if(BUILD_TESTING)
  file(GLOB FSAI_CONTROL_TESTS CONFIGURE_DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/test/*.c")
  foreach(test_src ${FSAI_CONTROL_TESTS})
    get_filename_component(test_name ${test_src} NAME_WE)
    add_executable(${test_name} ${test_src})
    target_link_libraries(${test_name} PRIVATE fsai_control)
    add_test(NAME ${test_name} COMMAND ${test_name})
  endforeach()
endif()
