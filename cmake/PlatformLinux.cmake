if(NOT DEFINED FSAI_PLATFORM)
  set(FSAI_PLATFORM "linux" CACHE STRING "Target platform" FORCE)
endif()

add_compile_definitions(FSAI_PLATFORM_LINUX)
