if(NOT DEFINED FSAI_PLATFORM)
  set(FSAI_PLATFORM "mac" CACHE STRING "Target platform" FORCE)
endif()

add_compile_definitions(FSAI_PLATFORM_MAC)
