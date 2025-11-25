#pragma once

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include <SDL.h>

#include "Graphics.h"
#include "WorldConfig.hpp"
#include "gui_world_adapter.hpp"
#include "io_bus.hpp"
#include "runtime_telemetry.hpp"
#include "sim/architecture/IWorldView.hpp"
#include "sim/architecture/WorldDebug.hpp"

namespace fsai::io::camera::sim_stereo {
class SimStereoSource;
struct SimConeInstance;
}  // namespace fsai::io::camera::sim_stereo

namespace fsai::sim::world {

class WorldRenderAdapter {
 public:
  WorldRenderAdapter(const WorldRendererConfig& config,
                     const fsai::world::IWorldView& world_view,
                     fsai::io::IoBus& io_bus);
  ~WorldRenderAdapter();

  WorldRenderAdapter(const WorldRenderAdapter&) = delete;
  WorldRenderAdapter& operator=(const WorldRenderAdapter&) = delete;

  bool Initialize();
  void Shutdown();

  void HandleEvent(const SDL_Event& event);

  void Render(uint64_t now_ns,
              const fsai::sim::app::RuntimeTelemetry& telemetry);

  SDL_Window* window() const;
  SDL_Renderer* renderer() const;

  bool window_enabled() const { return window_initialized_; }
  bool stereo_enabled() const { return static_cast<bool>(stereo_source_); }

  void Present();

 private:
  void drawScene(const fsai::sim::app::GuiWorldSnapshot& snapshot,
                 const fsai::sim::app::RuntimeTelemetry& telemetry,
                 const std::optional<fsai::world::WorldDebugPacket>& debug_packet);
  void publishStereoFrame(const fsai::sim::app::GuiWorldSnapshot& snapshot,
                          uint64_t now_ns);

  WorldRendererConfig config_{};
  const fsai::world::IWorldView& world_view_;
  fsai::io::IoBus& io_bus_;
  fsai::sim::app::GuiWorldAdapter gui_adapter_;
  Graphics graphics_{};
  bool window_initialized_{false};
  bool initialized_{false};
  std::unique_ptr<fsai::io::camera::sim_stereo::SimStereoSource> stereo_source_;
  std::vector<fsai::io::camera::sim_stereo::SimConeInstance> cone_instances_;
};

}  // namespace fsai::sim::world
