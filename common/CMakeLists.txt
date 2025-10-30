file(GLOB_RECURSE FSAI_COMMON_SOURCES CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.c"
  "${CMAKE_CURRENT_SOURCE_DIR}/src/*.cpp")
file(GLOB_RECURSE FSAI_COMMON_HEADERS CONFIGURE_DEPENDS
  "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.h"
  "${CMAKE_CURRENT_SOURCE_DIR}/include/**/*.hpp")

add_library(fsai_common STATIC ${FSAI_COMMON_SOURCES})

set(_fsai_common_include_dirs "${CMAKE_CURRENT_SOURCE_DIR}/include")
foreach(header ${FSAI_COMMON_HEADERS})
  get_filename_component(dir "${header}" DIRECTORY)
  list(APPEND _fsai_common_include_dirs "${dir}")
endforeach()
list(REMOVE_DUPLICATES _fsai_common_include_dirs)

target_include_directories(fsai_common
  PUBLIC
    ${_fsai_common_include_dirs})

target_link_libraries(fsai_common
  PUBLIC
    Eigen3::Eigen
    Threads::Threads)
