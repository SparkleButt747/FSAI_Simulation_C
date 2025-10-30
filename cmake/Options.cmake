option(FSAI_BUILD_APPS "Build FS-AI application entrypoints" ON)
option(FSAI_BUILD_VISION "Build vision components" OFF)
option(FSAI_ENABLE_TESTS "Build unit and integration tests" ON)

if(FSAI_ENABLE_TESTS)
  set(BUILD_TESTING ON CACHE BOOL "Enable CTest" FORCE)
else()
  set(BUILD_TESTING OFF CACHE BOOL "Enable CTest" FORCE)
endif()
