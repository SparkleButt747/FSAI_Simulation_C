#include "WorldRenderAdapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "budget.h"
#include "centerline.hpp"
#include "logging.hpp"
#include "sim_stereo_source.hpp"
#include "sim/cone_constants.hpp"
#include "sim/integration/provider_registry.hpp"
#include "sim/world/TrackTypes.hpp"
#include "types.h"

namespace fsai::sim::world {
namespace {
constexpr float kConeDisplayScale = 12.0f;

void DrawConeMarker(Graphics* graphics, int center_x, int center_y,
                    float base_width_m, const SDL_Color& color) {
  if (graphics == nullptr || graphics->renderer == nullptr) {
    return;
  }

  const float base_radius_px =
      0.5f * base_width_m * K_RENDER_SCALE * kConeDisplayScale;
  const int radius_px =
      std::max(1, static_cast<int>(std::lround(base_radius_px)));

  SDL_SetRenderDrawColor(graphics->renderer, color.r, color.g, color.b,
                         color.a);
  Graphics_DrawFilledCircle(graphics, center_x, center_y, radius_px);
}
}  // namespace

WorldRenderAdapter::WorldRenderAdapter(const WorldRendererConfig& config,
                                       const fsai::world::IWorldView& world_view,
                                       fsai::io::IoBus& io_bus)
    : config_(config),
      world_view_(world_view),
      io_bus_(io_bus),
      gui_adapter_(world_view_) {}

WorldRenderAdapter::~WorldRenderAdapter() { Shutdown(); }

bool WorldRenderAdapter::Initialize() {
  if (initialized_) {
    return true;
  }

  if (config_.enable_window) {
    if (Graphics_Init(&graphics_, config_.window_title.c_str(),
                      config_.window_width, config_.window_height) != 0) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kError,
                           "Graphics initialization failed for window '%s'",
                           config_.window_title.c_str());
      return false;
    }
    window_initialized_ = true;
  }

  if (config_.publish_stereo_frames) {
    fsai::sim::integration::registerBuiltInStereoProviders();
    auto factory =
        fsai::sim::integration::lookupStereoProvider(config_.stereo_provider);
    if (!factory) {
      fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                           "Stereo provider '%s' not registered; disabling stereo",
                           config_.stereo_provider.c_str());
    } else {
      stereo_source_ = factory();
      if (!stereo_source_) {
        fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                             "Stereo provider '%s' failed to construct; disabling stereo",
                             config_.stereo_provider.c_str());
      }
    }
  }

  initialized_ = true;
  return true;
}

void WorldRenderAdapter::Shutdown() {
  stereo_source_.reset();
  if (window_initialized_) {
    Graphics_Cleanup(&graphics_);
    window_initialized_ = false;
  }
  initialized_ = false;
}

void WorldRenderAdapter::HandleEvent(const SDL_Event& event) {
  if (!window_initialized_) {
    return;
  }
  Graphics_HandleWindowEvent(&graphics_, &event);
}

void WorldRenderAdapter::Render(
    uint64_t now_ns, const fsai::sim::app::RuntimeTelemetry& telemetry) {
  if (!window_initialized_ && !stereo_source_) {
    return;
  }

  const auto snapshot = gui_adapter_.snapshot();
  const auto debug_packet = io_bus_.latest_world_debug();
  if (window_initialized_) {
    drawScene(snapshot, telemetry, debug_packet);
  }
  if (stereo_source_) {
    publishStereoFrame(snapshot, now_ns);
  }
}

SDL_Window* WorldRenderAdapter::window() const {
  return window_initialized_ ? graphics_.window : nullptr;
}

SDL_Renderer* WorldRenderAdapter::renderer() const {
  return window_initialized_ ? graphics_.renderer : nullptr;
}

void WorldRenderAdapter::Present() {
  if (!window_initialized_) {
    return;
  }
  Graphics_Present(&graphics_);
}

void WorldRenderAdapter::drawScene(
    const fsai::sim::app::GuiWorldSnapshot& snapshot,
    const fsai::sim::app::RuntimeTelemetry& telemetry,
    const std::optional<fsai::world::WorldDebugPacket>& debug_packet) {
  (void)telemetry;
  fsai::time::SimulationStageTimer render_timer("renderer");

  Graphics_Clear(&graphics_);
  if (config_.show_debug_overlays) {
    Graphics_DrawGrid(&graphics_, 50);
  }

  const auto& start_cones =
      debug_packet ? debug_packet->start_cones : snapshot.start_cones;
  const auto& left_cones =
      debug_packet ? debug_packet->left_cones : snapshot.left_cones;
  const auto& right_cones =
      debug_packet ? debug_packet->right_cones : snapshot.right_cones;
  const auto& checkpoints =
      debug_packet ? debug_packet->checkpoints : snapshot.checkpoints;
  const std::size_t gate_count =
      std::min(left_cones.size(), right_cones.size());

  if (gate_count > 0) {
    for (std::size_t i = 0; i < gate_count; ++i) {
      const bool is_current_gate = (i == 0);
      const SDL_Color color = is_current_gate
                                  ? SDL_Color{200, 0, 200, 255}
                                  : SDL_Color{120, 120, 200, 180};
      const auto& left = left_cones[i];
      const auto& right = right_cones[i];

      const float left_x = left.x * K_RENDER_SCALE + graphics_.width / 2.0f;
      const float left_y = left.z * K_RENDER_SCALE + graphics_.height / 2.0f;
      const float right_x =
          right.x * K_RENDER_SCALE + graphics_.width / 2.0f;
      const float right_y =
          right.z * K_RENDER_SCALE + graphics_.height / 2.0f;

      SDL_SetRenderDrawColor(graphics_.renderer, color.r, color.g, color.b,
                             color.a);
      SDL_RenderDrawLineF(graphics_.renderer, left_x, left_y, right_x,
                          right_y);

      if (is_current_gate && config_.show_debug_overlays) {
        const float thickness = std::max(1.5f, K_RENDER_SCALE * 0.15f);
        const float dx = right_x - left_x;
        const float dy = right_y - left_y;
        const float length = std::hypot(dx, dy);
        if (length > std::numeric_limits<float>::epsilon()) {
          const float nx = -dy / length;
          const float ny = dx / length;
          const float offset_x = nx * thickness * 0.5f;
          const float offset_y = ny * thickness * 0.5f;
          SDL_RenderDrawLineF(graphics_.renderer, left_x + offset_x,
                              left_y + offset_y, right_x + offset_x,
                              right_y + offset_y);
          SDL_RenderDrawLineF(graphics_.renderer, left_x - offset_x,
                              left_y - offset_y, right_x - offset_x,
                              right_y - offset_y);
        }
      }
    }
  } else {
    if (!checkpoints.empty()) {
      SDL_SetRenderDrawColor(graphics_.renderer, 200, 0, 200, 255);
      Graphics_DrawFilledCircle(
          &graphics_,
          static_cast<int>(checkpoints.front().x * K_RENDER_SCALE +
                           graphics_.width / 2.0f),
          static_cast<int>(checkpoints.front().z * K_RENDER_SCALE +
                           graphics_.height / 2.0f),
          static_cast<int>(K_RENDER_SCALE));
    }
  }

  const auto& lookahead = snapshot.lookahead;
  const bool show_overlays = config_.show_debug_overlays;
  const SDL_Color start_color{255, 140, 0, 255};
  for (const auto& cone : start_cones) {
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics_.height / 2.0f);
    DrawConeMarker(&graphics_, cone_x, cone_y,
                   fsai::sim::kLargeConeRadiusMeters * 2.0f, start_color);
  }

  const SDL_Color left_base{255, 214, 0, 255};
  for (size_t i = 0; i < left_cones.size(); ++i) {
    SDL_Color color = left_base;
    if (show_overlays) {
      if (i == 0) {
        color = SDL_Color{0, 255, 0, 255};
      } else if (static_cast<int>(i) == lookahead.speed) {
        color = SDL_Color{255, 255, 0, 255};
      } else if (static_cast<int>(i) == lookahead.steer) {
        color = SDL_Color{255, 0, 255, 255};
      }
    }
    const auto& cone = left_cones[i];
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics_.height / 2.0f);
    DrawConeMarker(&graphics_, cone_x, cone_y,
                   fsai::sim::kSmallConeRadiusMeters * 2.0f, color);
  }

  const SDL_Color right_base{0, 102, 204, 255};
  for (size_t i = 0; i < right_cones.size(); ++i) {
    SDL_Color color = right_base;
    if (show_overlays) {
      if (i == 0) {
        color = SDL_Color{0, 255, 0, 255};
      } else if (static_cast<int>(i) == lookahead.speed) {
        color = SDL_Color{255, 255, 0, 255};
      } else if (static_cast<int>(i) == lookahead.steer) {
        color = SDL_Color{255, 0, 255, 255};
      }
    }
    const auto& cone = right_cones[i];
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics_.height / 2.0f);
    DrawConeMarker(&graphics_, cone_x, cone_y,
                   fsai::sim::kSmallConeRadiusMeters * 2.0f, color);
  }

  const auto& transform = snapshot.vehicle_transform;
  const float car_screen_x = transform.position.x * K_RENDER_SCALE +
                             graphics_.width / 2.0f;
  const float car_screen_y = transform.position.z * K_RENDER_SCALE +
                             graphics_.height / 2.0f;
  const float car_radius = 2.0f * K_RENDER_SCALE;
  Graphics_DrawCar(&graphics_, car_screen_x, car_screen_y, car_radius,
                   transform.yaw);

  if (show_overlays && debug_packet && !debug_packet->detections.empty()) {
    for (const auto& cone : debug_packet->detections) {
      int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                    graphics_.width / 2.0f);
      int cone_y = static_cast<int>(cone.y * K_RENDER_SCALE +
                                    graphics_.height / 2.0f);
      if (cone.side == FSAI_CONE_LEFT) {
        SDL_SetRenderDrawColor(graphics_.renderer, 5, 200, 5, 255);
      } else if (cone.side == FSAI_CONE_RIGHT) {
        SDL_SetRenderDrawColor(graphics_.renderer, 200, 5, 5, 255);
      } else {
        SDL_SetRenderDrawColor(graphics_.renderer, 150, 150, 150, 250);
      }
      Graphics_DrawFilledCircle(&graphics_, cone_x, cone_y, 5);
    }
  }

  if (show_overlays) {
    auto to_cones = [](const std::vector<Vector3>& positions, ConeType type) {
      std::vector<Cone> cones;
      cones.reserve(positions.size());
      for (const auto& pos : positions) {
        Cone cone{};
        cone.position = pos;
        cone.type = type;
        cones.push_back(cone);
      }
      return cones;
    };

    std::vector<std::pair<Vector2, Vector2>> triangulation_edges =
        getVisibleTriangulationEdges(
            snapshot.vehicle_state,
            to_cones(left_cones, ConeType::Left),
            to_cones(right_cones, ConeType::Right))
            .second;
    for (const auto& edge : triangulation_edges) {
      Graphics_DrawSegment(&graphics_, edge.first.x, edge.first.y,
                           edge.second.x, edge.second.y, 50, 0, 255);
    }
    if (debug_packet) {
      for (const auto& edge : debug_packet->controller_path_edges) {
        Graphics_DrawSegment(&graphics_, edge.first.x, edge.first.y,
                             edge.second.x, edge.second.y, 255, 50, 50);
      }
    }
  }
}

void WorldRenderAdapter::publishStereoFrame(
    const fsai::sim::app::GuiWorldSnapshot& snapshot, uint64_t now_ns) {
  if (!stereo_source_) {
    return;
  }

  const auto& transform = snapshot.vehicle_transform;
  stereo_source_->setBodyPose(transform.position.x, transform.position.y,
                              transform.position.z, transform.yaw);

  cone_instances_.clear();
  const std::size_t cone_count = snapshot.start_cones.size() +
                                 snapshot.left_cones.size() +
                                 snapshot.right_cones.size();
  cone_instances_.reserve(cone_count);

  const float color_scale = 1.0f / 255.0f;
  auto makeColor = [color_scale](int r, int g, int b) {
    return std::array<float, 3>{r * color_scale, g * color_scale,
                                b * color_scale};
  };

  const auto start_body = makeColor(255, 140, 0);
  const auto start_stripe = makeColor(255, 255, 255);
  const auto left_body = makeColor(255, 214, 0);
  const auto left_stripe = makeColor(50, 50, 50);
  const auto right_body = makeColor(0, 102, 204);
  const auto right_stripe = makeColor(255, 255, 255);

  auto appendCone = [&](const Vector3& pos, float base_width, float height,
                        const std::array<float, 3>& body_color,
                        const std::array<float, 3>& stripe_color,
                        int stripes) {
    fsai::io::camera::sim_stereo::SimConeInstance instance{};
    instance.position = {pos.x, pos.y, pos.z};
    instance.base_width = base_width;
    instance.height = height;
    instance.body_color = body_color;
    instance.stripe_color = stripe_color;
    instance.stripe_count = stripes;
    cone_instances_.push_back(instance);
  };

  for (const auto& cone : snapshot.start_cones) {
    appendCone(cone, fsai::sim::kLargeConeRadiusMeters * 2.0f,
               fsai::sim::kLargeConeHeightMeters, start_body, start_stripe, 2);
  }
  for (const auto& cone : snapshot.left_cones) {
    appendCone(cone, fsai::sim::kSmallConeRadiusMeters * 2.0f,
               fsai::sim::kSmallConeHeightMeters, left_body, left_stripe, 1);
  }
  for (const auto& cone : snapshot.right_cones) {
    appendCone(cone, fsai::sim::kSmallConeRadiusMeters * 2.0f,
               fsai::sim::kSmallConeHeightMeters, right_body, right_stripe, 1);
  }

  stereo_source_->setCones(cone_instances_);
  const FsaiStereoFrame& frame = stereo_source_->capture(now_ns);
  io_bus_.publish_stereo_frame(frame);
}

}  // namespace fsai::sim::world
