#pragma once

#include <functional>
#include <memory>
#include <optional>

#include <SDL.h>

#include "Graphics.h"
#include "gui_world_adapter.hpp"
#include "runtime_telemetry.hpp"
#include "sim_stereo_source.hpp"
#include "stereo_display.hpp"
#include "vision/frame_ring_buffer.hpp"
#include "io_bus.hpp"
#include "sim/architecture/IWorldView.hpp"

namespace fsai::sim::world {

struct WorldRenderConfig {
  bool graphics_enabled{true};
  bool simstereo_enabled{true};
  bool stereo_display{false};
  bool edge_preview_enabled{true};
  bool detection_preview_enabled{true};
  bool test_mode{false};
  int window_width{800};
  int window_height{600};
  int stereo_width{640};
  int stereo_height{480};
};

WorldRenderConfig LoadWorldRenderConfig(const std::string& path);

class WorldRenderAdapter {
 public:
  WorldRenderAdapter(WorldRenderConfig config, fsai::io::IoBus& io,
                     const fsai::world::IWorldView& world);
  ~WorldRenderAdapter();

  WorldRenderAdapter(const WorldRenderAdapter&) = delete;
  WorldRenderAdapter& operator=(const WorldRenderAdapter&) = delete;

  bool Initialize();
  void Shutdown();

  bool headless() const { return headless_; }

  SDL_Renderer* renderer();

  void set_stereo_frame_buffer(
      const std::shared_ptr<fsai::vision::FrameRingBuffer>& buffer);
  void set_stereo_display(
      std::unique_ptr<fsai::sim::integration::StereoDisplay> display);

  bool PumpEvents(const std::function<bool(const SDL_Event&)>& handler);
  void BeginFrame();
  void Render(const fsai::sim::app::RuntimeTelemetry& telemetry,
              uint64_t now_ns);

 private:
  void renderWorld(const fsai::sim::app::RuntimeTelemetry& telemetry);
  void renderStereo(uint64_t now_ns);
  bool initGraphics();
  void cleanupGraphics();
  bool initStereo();
  void cleanupStereo();

  WorldRenderConfig config_{};
  fsai::io::IoBus& io_;
  const fsai::world::IWorldView& world_view_;
  fsai::sim::app::GuiWorldAdapter gui_adapter_;

  Graphics graphics_{};
  bool headless_{false};
  bool imgui_ready_{false};
  bool graphics_ready_{false};
  bool running_{true};

  std::unique_ptr<fsai::io::camera::sim_stereo::SimStereoSource>
      stereo_source_{};
  std::shared_ptr<fsai::vision::FrameRingBuffer> stereo_frame_buffer_{};
  std::unique_ptr<fsai::sim::integration::StereoDisplay> stereo_display_{};
  std::vector<fsai::io::camera::sim_stereo::SimConeInstance> stereo_cones_{};
};

}  // namespace fsai::sim::world

