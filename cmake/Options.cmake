option(FSAI_BUILD_VISION "Build experimental vision targets" OFF)
option(FSAI_ENABLE_APPS "Build application entrypoints" ON)
option(FSAI_ENABLE_TESTS "Enable unit/integration tests" ON)

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "Build type" FORCE)
endif()
