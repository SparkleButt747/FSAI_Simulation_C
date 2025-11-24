#include "WorldRenderAdapter.hpp"

#include <array>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <string>

#include <yaml-cpp/yaml.h>

#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"
#include "budget.h"
#include "centerline.hpp"
#include "imgui.h"
#include "logging.hpp"
#include "cone_constants.hpp"

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

void DrawWorldScene(Graphics* graphics,
                    const fsai::sim::app::GuiWorldSnapshot& world,
                    const fsai::sim::app::RuntimeTelemetry& telemetry) {
  (void)telemetry;
  fsai::time::SimulationStageTimer render_timer("renderer");
  Graphics_Clear(graphics);
  Graphics_DrawGrid(graphics, 50);

  const auto& left_cones = world.left_cones;
  const auto& right_cones = world.right_cones;
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

      const float left_x = left.x * K_RENDER_SCALE + graphics->width / 2.0f;
      const float left_y = left.z * K_RENDER_SCALE + graphics->height / 2.0f;
      const float right_x =
          right.x * K_RENDER_SCALE + graphics->width / 2.0f;
      const float right_y =
          right.z * K_RENDER_SCALE + graphics->height / 2.0f;

      SDL_SetRenderDrawColor(graphics->renderer, color.r, color.g, color.b,
                             color.a);
      SDL_RenderDrawLineF(graphics->renderer, left_x, left_y, right_x,
                          right_y);

      if (is_current_gate) {
        const float thickness = std::max(1.5f, K_RENDER_SCALE * 0.15f);
        const float dx = right_x - left_x;
        const float dy = right_y - left_y;
        const float length = std::hypot(dx, dy);
        if (length > std::numeric_limits<float>::epsilon()) {
          const float nx = -dy / length;
          const float ny = dx / length;
          const float offset_x = nx * thickness * 0.5f;
          const float offset_y = ny * thickness * 0.5f;
          SDL_RenderDrawLineF(graphics->renderer, left_x + offset_x,
                              left_y + offset_y, right_x + offset_x,
                              right_y + offset_y);
          SDL_RenderDrawLineF(graphics->renderer, left_x - offset_x,
                              left_y - offset_y, right_x - offset_x,
                              right_y - offset_y);
        }
      }
    }
  } else {
    const auto& checkpoints = world.checkpoints;
    if (!checkpoints.empty()) {
      SDL_SetRenderDrawColor(graphics->renderer, 200, 0, 200, 255);
      Graphics_DrawFilledCircle(
          graphics,
          static_cast<int>(checkpoints.front().x * K_RENDER_SCALE +
                           graphics->width / 2.0f),
          static_cast<int>(checkpoints.front().z * K_RENDER_SCALE +
                           graphics->height / 2.0f),
          static_cast<int>(K_RENDER_SCALE));
    }
  }

  const auto& lookahead = world.lookahead;

  const auto& start_cones = world.start_cones;
  const SDL_Color start_color{255, 140, 0, 255};
  for (const auto& cone : start_cones) {
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
    DrawConeMarker(graphics, cone_x, cone_y,
                  fsai::sim::kLargeConeRadiusMeters * 2.0f, start_color);
  }
  const SDL_Color left_base{255, 214, 0, 255};
  for (size_t i = 0; i < world.left_cones.size(); ++i) {
    SDL_Color color = left_base;
    if (i == 0) {
      color = SDL_Color{0, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.speed) {
      color = SDL_Color{255, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.steer) {
      color = SDL_Color{255, 0, 255, 255};
    }
    const auto& cone = world.left_cones[i];
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
    DrawConeMarker(graphics, cone_x, cone_y,
                  fsai::sim::kSmallConeRadiusMeters * 2.0f, color);
  }
  const SDL_Color right_base{0, 102, 204, 255};
  for (size_t i = 0; i < world.right_cones.size(); ++i) {
    SDL_Color color = right_base;
    if (i == 0) {
      color = SDL_Color{0, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.speed) {
      color = SDL_Color{255, 255, 0, 255};
    } else if (static_cast<int>(i) == lookahead.steer) {
      color = SDL_Color{255, 0, 255, 255};
    }
    const auto& cone = world.right_cones[i];
    const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                        graphics->width / 2.0f);
    const int cone_y = static_cast<int>(cone.z * K_RENDER_SCALE +
                                        graphics->height / 2.0f);
    DrawConeMarker(graphics, cone_x, cone_y,
                  fsai::sim::kSmallConeRadiusMeters * 2.0f, color);
  }

  const auto& transform = world.vehicle_transform;
  const float car_screen_x = transform.position.x * K_RENDER_SCALE +
                             graphics->width / 2.0f;
  const float car_screen_y = transform.position.z * K_RENDER_SCALE +
                             graphics->height / 2.0f;
  const float car_radius = 2.0f * K_RENDER_SCALE;
  Graphics_DrawCar(graphics, car_screen_x, car_screen_y, car_radius,
                   transform.yaw);

  if (world.detections != nullptr) {
    for (const auto& cone : *world.detections) {
      const int cone_x = static_cast<int>(cone.x * K_RENDER_SCALE +
                                          graphics->width / 2.0f);
      const int cone_y = static_cast<int>(cone.y * K_RENDER_SCALE +
                                          graphics->height / 2.0f);
      if (cone.side == FSAI_CONE_LEFT) {
        SDL_SetRenderDrawColor(graphics->renderer, 5, 200, 5, 255);
      } else if (cone.side == FSAI_CONE_RIGHT) {
        SDL_SetRenderDrawColor(graphics->renderer, 200, 5, 5, 255);
      } else if (cone.side == FSAI_CONE_UNKNOWN) {
        SDL_SetRenderDrawColor(graphics->renderer, 150, 150, 150, 250);
      }
      Graphics_DrawFilledCircle(graphics, cone_x, cone_y, 5);
    }
  }
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

  std::vector<std::pair<Vector2, Vector2>> triangulationEdges =
      getVisibleTriangulationEdges(world.vehicle_state,
                                   to_cones(world.left_cones, ConeType::Left),
                                   to_cones(world.right_cones, ConeType::Right))
          .second;
  for (auto edge : triangulationEdges) {
    Graphics_DrawSegment(graphics, edge.first.x, edge.first.y, edge.second.x,
                         edge.second.y, 50, 0, 255);
  }
  for (auto edge : world.best_path_edges) {
    Graphics_DrawSegment(graphics, edge.first.x, edge.first.y, edge.second.x,
                         edge.second.y, 255, 50, 50);
  }
}

fsai::io::camera::sim_stereo::SimStereoConfig DefaultStereoConfig(
    const WorldRenderConfig& cfg) {
  fsai::io::camera::sim_stereo::SimStereoConfig stereo_cfg;
  stereo_cfg.width = cfg.stereo_width;
  stereo_cfg.height = cfg.stereo_height;
  stereo_cfg.near_plane = 0.1f;
  stereo_cfg.far_plane = 100.0f;
  stereo_cfg.intrinsics.fx = static_cast<float>(cfg.stereo_width) * 0.625f;
  stereo_cfg.intrinsics.fy = static_cast<float>(cfg.stereo_height) * 0.833f;
  stereo_cfg.intrinsics.cx = static_cast<float>(cfg.stereo_width) * 0.5f;
  stereo_cfg.intrinsics.cy = static_cast<float>(cfg.stereo_height) * 0.5f;
  stereo_cfg.right_extrinsics.t[0] = 0.2f;
  stereo_cfg.right_extrinsics.t[2] = 0.0f;
  stereo_cfg.left_extrinsics.R[0] = stereo_cfg.left_extrinsics.R[4] =
      stereo_cfg.left_extrinsics.R[8] = 1.0f;
  stereo_cfg.right_extrinsics.R[0] = stereo_cfg.right_extrinsics.R[4] =
      stereo_cfg.right_extrinsics.R[8] = 1.0f;
  return stereo_cfg;
}

}  // namespace

WorldRenderConfig LoadWorldRenderConfig(const std::string& path) {
  WorldRenderConfig cfg{};
  try {
    YAML::Node root = YAML::LoadFile(path);
    if (auto graphics = root["graphics"]) {
      cfg.graphics_enabled = graphics["enabled"].as<bool>(cfg.graphics_enabled);
      cfg.window_width = graphics["width"].as<int>(cfg.window_width);
      cfg.window_height = graphics["height"].as<int>(cfg.window_height);
      cfg.edge_preview_enabled =
          graphics["edge_preview"].as<bool>(cfg.edge_preview_enabled);
      cfg.detection_preview_enabled =
          graphics["detection_preview"].as<bool>(cfg.detection_preview_enabled);
    }
    if (auto stereo = root["simstereo"]) {
      cfg.simstereo_enabled = stereo["enabled"].as<bool>(cfg.simstereo_enabled);
      cfg.stereo_display = stereo["display"].as<bool>(cfg.stereo_display);
      cfg.stereo_width = stereo["width"].as<int>(cfg.stereo_width);
      cfg.stereo_height = stereo["height"].as<int>(cfg.stereo_height);
    }
    if (auto test_mode = root["test_mode"]) {
      cfg.test_mode = test_mode.as<bool>(cfg.test_mode);
    }
  } catch (const std::exception&) {
  }

  if (cfg.test_mode) {
    cfg.graphics_enabled = false;
    cfg.simstereo_enabled = false;
  }
  return cfg;
}

WorldRenderAdapter::WorldRenderAdapter(WorldRenderConfig config,
                                       fsai::io::IoBus& io,
                                       const fsai::world::IWorldView& world)
    : config_(config),
      io_(io),
      world_view_(world),
      gui_adapter_(world) {}

WorldRenderAdapter::~WorldRenderAdapter() { Shutdown(); }

bool WorldRenderAdapter::Initialize() {
  headless_ = config_.test_mode || !config_.graphics_enabled;
  if (headless_) {
    return true;
  }
  if (!initGraphics()) {
    return false;
  }
  return initStereo();
}

void WorldRenderAdapter::Shutdown() {
  cleanupStereo();
  cleanupGraphics();
}

SDL_Renderer* WorldRenderAdapter::renderer() {
  return headless_ ? nullptr : graphics_.renderer;
}

void WorldRenderAdapter::set_stereo_frame_buffer(
    const std::shared_ptr<fsai::vision::FrameRingBuffer>& buffer) {
  stereo_frame_buffer_ = buffer;
}

void WorldRenderAdapter::set_stereo_display(
    std::unique_ptr<fsai::sim::integration::StereoDisplay> display) {
  stereo_display_ = std::move(display);
}

bool WorldRenderAdapter::PumpEvents(
    const std::function<bool(const SDL_Event&)>& handler) {
  if (headless_) {
    return true;
  }
  SDL_Event event;
  while (SDL_PollEvent(&event)) {
    ImGui_ImplSDL2_ProcessEvent(&event);
    Graphics_HandleWindowEvent(&graphics_, &event);
    if (handler && !handler(event)) {
      running_ = false;
    }
    if (event.type == SDL_QUIT) {
      running_ = false;
    }
  }
  return running_;
}

void WorldRenderAdapter::BeginFrame() {
  if (headless_) {
    return;
  }
  ImGui_ImplSDLRenderer2_NewFrame();
  ImGui_ImplSDL2_NewFrame();
  ImGui::NewFrame();
}

void WorldRenderAdapter::Render(
    const fsai::sim::app::RuntimeTelemetry& telemetry, uint64_t now_ns) {
  if (headless_) {
    return;
  }
  renderWorld(telemetry);
  ImGui::Render();
  ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(),
                                        graphics_.renderer);
  Graphics_Present(&graphics_);
  renderStereo(now_ns);
}

void WorldRenderAdapter::renderWorld(
    const fsai::sim::app::RuntimeTelemetry& telemetry) {
  const auto snapshot = gui_adapter_.snapshot();
  DrawWorldScene(&graphics_, snapshot, telemetry);
}

void WorldRenderAdapter::renderStereo(uint64_t now_ns) {
  if (!stereo_source_) {
    return;
  }
  const auto& transform = world_view_.vehicle_transform();
  stereo_source_->setBodyPose(transform.position.x, transform.position.y,
                              transform.position.z, transform.yaw);

  stereo_cones_.clear();
  stereo_cones_.reserve(world_view_.left_cones().size() +
                        world_view_.right_cones().size() +
                        world_view_.start_cones().size());

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

  auto appendCone = [&](const Vector3& pos, ConeType type) {
    fsai::io::camera::sim_stereo::SimConeInstance instance{};
    instance.position = {pos.x, pos.y, pos.z};
    switch (type) {
      case ConeType::Start:
        instance.base_width = fsai::sim::kLargeConeRadiusMeters * 2.0f;
        instance.height = fsai::sim::kLargeConeHeightMeters;
        instance.body_color = start_body;
        instance.stripe_color = start_stripe;
        instance.stripe_count = 2;
        break;
      case ConeType::Left:
        instance.base_width = fsai::sim::kSmallConeRadiusMeters * 2.0f;
        instance.height = fsai::sim::kSmallConeHeightMeters;
        instance.body_color = left_body;
        instance.stripe_color = left_stripe;
        instance.stripe_count = 1;
        break;
      case ConeType::Right:
        instance.base_width = fsai::sim::kSmallConeRadiusMeters * 2.0f;
        instance.height = fsai::sim::kSmallConeHeightMeters;
        instance.body_color = right_body;
        instance.stripe_color = right_stripe;
        instance.stripe_count = 1;
        break;
    }
    stereo_cones_.push_back(instance);
  };

  for (const auto& cone : world_view_.left_cones()) {
    appendCone(cone, ConeType::Left);
  }
  for (const auto& cone : world_view_.right_cones()) {
    appendCone(cone, ConeType::Right);
  }
  for (const auto& cone : world_view_.start_cones()) {
    appendCone(cone, ConeType::Start);
  }
  stereo_source_->setCones(stereo_cones_);

  const FsaiStereoFrame& frame = stereo_source_->capture(now_ns);
  io_.publish_stereo_frame(frame);
  if (stereo_frame_buffer_) {
    while (!stereo_frame_buffer_->tryPush(frame)) {
      if (!stereo_frame_buffer_->tryPop().has_value()) {
        break;
      }
    }
  }
  if (stereo_display_) {
    stereo_display_->present(frame);
  }
}

bool WorldRenderAdapter::initGraphics() {
  if (Graphics_Init(&graphics_, "Car Simulation 2D", config_.window_width,
                    config_.window_height) != 0) {
    return false;
  }

  IMGUI_CHECKVERSION();
  ImGui::CreateContext();
  ImGui::StyleColorsDark();
  if (!ImGui_ImplSDL2_InitForSDLRenderer(graphics_.window, graphics_.renderer)) {
    ImGui::DestroyContext();
    Graphics_Cleanup(&graphics_);
    return false;
  }
  if (!ImGui_ImplSDLRenderer2_Init(graphics_.renderer)) {
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    Graphics_Cleanup(&graphics_);
    return false;
  }
  imgui_ready_ = true;
  graphics_ready_ = true;
  return true;
}

void WorldRenderAdapter::cleanupGraphics() {
  if (imgui_ready_) {
    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();
    imgui_ready_ = false;
  }
  if (graphics_ready_) {
    Graphics_Cleanup(&graphics_);
    graphics_.renderer = nullptr;
    graphics_.window = nullptr;
    graphics_ready_ = false;
  }
}

bool WorldRenderAdapter::initStereo() {
  if (!config_.simstereo_enabled) {
    return true;
  }

  try {
    stereo_source_ = std::make_unique<fsai::io::camera::sim_stereo::SimStereoSource>(
        DefaultStereoConfig(config_));
  } catch (const std::exception& e) {
    fsai::sim::log::Logf(fsai::sim::log::Level::kWarning,
                         "Sim stereo disabled: %s", e.what());
    stereo_source_.reset();
    return false;
  }

  if (config_.stereo_display) {
    stereo_display_ = std::make_unique<fsai::sim::integration::StereoDisplay>();
  }
  return true;
}

void WorldRenderAdapter::cleanupStereo() { stereo_source_.reset(); }

}  // namespace fsai::sim::world

