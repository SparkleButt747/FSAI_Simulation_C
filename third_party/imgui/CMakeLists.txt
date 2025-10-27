add_library(imgui STATIC
  imgui.cpp
  imgui_demo.cpp
  imgui_draw.cpp
  imgui_tables.cpp
  imgui_widgets.cpp
  backends/imgui_impl_sdl2.cpp
  backends/imgui_impl_sdlrenderer2.cpp
)

target_include_directories(imgui
  PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/backends
)

if(TARGET SDL2::SDL2)
  target_link_libraries(imgui PUBLIC SDL2::SDL2)
elseif(TARGET SDL2::SDL2-static)
  target_link_libraries(imgui PUBLIC SDL2::SDL2-static)
elseif(TARGET PkgConfig::SDL2)
  target_link_libraries(imgui PUBLIC PkgConfig::SDL2)
endif()
