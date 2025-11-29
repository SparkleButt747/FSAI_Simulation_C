#include "WorldRenderAdapter.hpp"

#include "../../include/logging.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <limits>
#include <string>
#include <utility>

#include "budget.h"
#include "centerline.hpp"
#include "sim_stereo_source.hpp"
#include "sim/cone_constants.hpp"
#include "sim/integration/provider_registry.hpp"
#include "sim/world/TrackTypes.hpp"
#include "types.h"

namespace fsai::sim::world {
namespace {
constexpr float kConeDisplayScale = 12.0f;
constexpr float kStereoCameraHeightOffset = 0.4f;

void DrawConeCrosshair(Graphics* graphics, int center_x, int center_y,
                       float base_width_m, float render_scale,
                       const SDL_Color& color) {
  if (graphics == nullptr || graphics->renderer == nullptr) {
    return;
  }

  const float base_radius_px =
      0.5f * base_width_m * render_scale * kConeDisplayScale;
  const int radius_px =
      std::max(2, static_cast<int>(std::lround(base_radius_px)));

  SDL_SetRenderDrawColor(graphics->renderer, color.r, color.g, color.b,
                         color.a);
  SDL_RenderDrawLine(graphics->renderer, center_x - radius_px, center_y,
                     center_x + radius_px, center_y);
  SDL_RenderDrawLine(graphics->renderer, center_x, center_y - radius_px,
                     center_x, center_y + radius_px);
}

void DrawCheckpointSquare(Graphics* graphics, int center_x, int center_y,
                          float size_px, const SDL_Color& color) {
  if (graphics == nullptr || graphics->renderer == nullptr) {
    return;
  }
  SDL_Rect rect{};
  rect.x = static_cast<int>(std::lround(center_x - size_px * 0.5f));
  rect.y = static_cast<int>(std::lround(center_y - size_px * 0.5f));
  rect.w = static_cast<int>(std::lround(size_px));
  rect.h = rect.w;
  SDL_SetRenderDrawColor(graphics->renderer, color.r, color.g, color.b,
                         color.a);
  SDL_RenderFillRect(graphics->renderer, &rect);
}
}  // namespace

// WORLD RENDERER: owns the graphics window, pushes stereo frames into Vision,
// and reads live World snapshots + IO debug packets so the GUI can annotate
// ground truth, detections, and controller state in one place.
WorldRenderAdapter::WorldRenderAdapter(const WorldRendererConfig& config,
                                       const fsai::world::IWorldView& world_view,
                                       fsai::io::IoBus& io_bus)
    : config_(config),
      world_view_(world_view),
      io_bus_(io_bus),
      gui_adapter_(world_view_) {
  render_scale_ = std::max(0.01f, config_.render_scale);
}

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

  // RENDER PASS: capture a consistent snapshot from the WorldView and combine
  // it with any IO-provided debug overlays (e.g., Control/Vision annotations)
  // before drawing the GUI or pushing stereo frames.
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
  const float render_scale = render_scale_;

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
  const auto& orange_cones =
      debug_packet ? debug_packet->orange_cones : snapshot.orange_cones;
  const auto& checkpoints =
      debug_packet ? debug_packet->checkpoints : snapshot.checkpoints;
  const auto& controller_path_edges =
      debug_packet ? debug_packet->controller_path_edges
                   : snapshot.controller_path_edges;
  const auto& triangulation_edges =
      debug_packet ? debug_packet->triangulation_edges
                   : snapshot.triangulation_edges;
  const auto& vision_detections =
      debug_packet ? debug_packet->detections : snapshot.detections;
  const std::size_t gate_count =
      std::min(left_cones.size(), right_cones.size());

  if (gate_count > 0) {
    for (std::size_t i = 0; i < gate_count; ++i) {
      const bool is_current_gate = (i == 0);
      const SDL_Color color = is_current_gate
                                  ? SDL_Color{255, 165, 0, 255}
                                  : SDL_Color{255, 200, 120, 190};
      const auto& left = left_cones[i];
      const auto& right = right_cones[i];

      const float left_x = left.x * render_scale + graphics_.width / 2.0f;
      const float left_y = left.z * render_scale + graphics_.height / 2.0f;
      const float right_x =
          right.x * render_scale + graphics_.width / 2.0f;
      const float right_y =
          right.z * render_scale + graphics_.height / 2.0f;

      SDL_SetRenderDrawColor(graphics_.renderer, color.r, color.g, color.b,
                             color.a);
      SDL_RenderDrawLineF(graphics_.renderer, left_x, left_y, right_x,
                          right_y);

      if (is_current_gate && config_.show_debug_overlays) {
        const float thickness = std::max(1.5f, render_scale * 0.15f);
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
      const SDL_Color checkpoint_color{210, 20, 20, 220};
      DrawCheckpointSquare(
          &graphics_,
          static_cast<int>(std::lround(checkpoints.front().x * render_scale +
                                        graphics_.width / 2.0f)),
          static_cast<int>(std::lround(checkpoints.front().z * render_scale +
                                        graphics_.height / 2.0f)),
          std::max(4.0f, render_scale * 1.5f), checkpoint_color);
    }
  }

  if (!checkpoints.empty()) {
    const SDL_Color checkpoint_color{210, 20, 20, 220};
    for (const auto& checkpoint : checkpoints) {
      const float cx = checkpoint.x * render_scale + graphics_.width / 2.0f;
      const float cy = checkpoint.z * render_scale + graphics_.height / 2.0f;
      DrawCheckpointSquare(&graphics_, static_cast<int>(std::lround(cx)),
                           static_cast<int>(std::lround(cy)),
                           std::max(4.0f, render_scale * 1.5f),
                           checkpoint_color);
    }
  }

  const auto& lookahead = snapshot.lookahead;
  const bool show_overlays = config_.show_debug_overlays;
  const SDL_Color start_color{255, 140, 0, 255};
  for (const auto& cone : start_cones) {
    const int cone_x = static_cast<int>(cone.x * render_scale +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * render_scale +
                                        graphics_.height / 2.0f);
    DrawConeCrosshair(&graphics_, cone_x, cone_y,
                      fsai::sim::kLargeConeRadiusMeters * 2.0f, render_scale,
                      start_color);
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
    const int cone_x = static_cast<int>(cone.x * render_scale +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * render_scale +
                                        graphics_.height / 2.0f);
    DrawConeCrosshair(&graphics_, cone_x, cone_y,
                      fsai::sim::kSmallConeRadiusMeters * 2.0f, render_scale,
                      color);
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
    const int cone_x = static_cast<int>(cone.x * render_scale +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * render_scale +
                                        graphics_.height / 2.0f);
    DrawConeCrosshair(&graphics_, cone_x, cone_y,
                      fsai::sim::kSmallConeRadiusMeters * 2.0f, render_scale,
                      color);
  }

  const SDL_Color orange_base{255, 165, 0, 255};
  for (size_t i = 0; i < orange_cones.size(); ++i) {
    SDL_Color color = orange_base;
    const auto& cone = orange_cones[i];
    const int cone_x = static_cast<int>(cone.x * render_scale +
                                        graphics_.width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * render_scale +
                                        graphics_.height / 2.0f);
    DrawConeCrosshair(&graphics_, cone_x, cone_y,
                      fsai::sim::kSmallConeRadiusMeters * 2.0f, render_scale,
                      color);
  }

  const auto& transform = snapshot.vehicle_transform;
  const float car_screen_x = transform.position.x * render_scale +
                             graphics_.width / 2.0f;
  const float car_screen_y = transform.position.z * render_scale +
                             graphics_.height / 2.0f;
  const float car_radius = 2.0f * render_scale;
  Graphics_DrawCar(&graphics_, car_screen_x, car_screen_y, car_radius,
                   transform.yaw);

  if (show_overlays && !vision_detections.empty()) {
    for (const auto& cone : vision_detections) {
      int cone_x = static_cast<int>(cone.x * render_scale +
                                    graphics_.width / 2.0f);
      int cone_y = static_cast<int>(cone.y * render_scale +
                                    graphics_.height / 2.0f);
      SDL_Color color{150, 150, 150, 230};
      if (cone.side == FSAI_CONE_LEFT) {
        color = SDL_Color{40, 205, 40, 240};
      } else if (cone.side == FSAI_CONE_RIGHT) {
        color = SDL_Color{215, 35, 35, 240};
      }
      DrawConeCrosshair(&graphics_, cone_x, cone_y,
                        fsai::sim::kSmallConeRadiusMeters * 1.5f, render_scale,
                        color);
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

    std::vector<std::pair<Vector2, Vector2>> snapshot_triangulation_edges = snapshot.triangulation_edges;
    for (const auto& edge : snapshot_triangulation_edges) {
      Graphics_DrawSegment(&graphics_, edge.first.x, edge.first.y,
                           edge.second.x, edge.second.y, render_scale, 50, 0,
                           255);
    }
    const SDL_Color path_color{0, 220, 200, 255};
    for (const auto& edge : controller_path_edges) {
      SDL_SetRenderDrawColor(graphics_.renderer, path_color.r, path_color.g,
                             path_color.b, path_color.a);
      Graphics_DrawSegment(&graphics_, edge.first.x, edge.first.y,
                           edge.second.x, edge.second.y, render_scale,
                           path_color.r, path_color.g, path_color.b);
    }
  }
}

void WorldRenderAdapter::publishStereoFrame(
    const fsai::sim::app::GuiWorldSnapshot& snapshot, uint64_t now_ns) {
  if (!stereo_source_) {
    return;
  }

  const auto& transform = snapshot.vehicle_transform;
  stereo_source_->setBodyPose(transform.position.x,
                              transform.position.y + kStereoCameraHeightOffset,
                              transform.position.z, transform.yaw);

  cone_instances_.clear();
  const std::size_t cone_count = snapshot.start_cones.size() +
                                 snapshot.left_cones.size() +
                                 snapshot.right_cones.size() +
                                 snapshot.orange_cones.size();
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
  const auto orange_body = makeColor(255, 165, 0);
  const auto orange_stripe = makeColor(255, 255, 255);

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
  for (const auto& cone : snapshot.orange_cones) {
    appendCone(cone, fsai::sim::kSmallConeRadiusMeters * 2.0f,
               fsai::sim::kSmallConeHeightMeters, orange_body, orange_stripe, 1);
  }

  stereo_source_->setCones(cone_instances_);
  const FsaiStereoFrame& frame = stereo_source_->capture(now_ns);
  io_bus_.publish_stereo_frame(frame);
}

}  // namespace fsai::sim::world
