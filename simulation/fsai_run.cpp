#include <array>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>

#include <SDL.h>

#include "Graphics.h"
#include "budget.h"
#include "fsai_clock.h"
#include "sim/integration/csv_logger.hpp"
#include "sim/integration/provider_registry.hpp"
#include "sim/integration/stereo_display.hpp"
#include "sim/integration/terminal_keyboard.hpp"
#include "sim_stereo_source.hpp"
#include "types.h"
#include "World.hpp"

namespace {
constexpr double kDefaultDt = 0.01;
constexpr int kWindowWidth = 800;
constexpr int kWindowHeight = 600;
constexpr int kReportIntervalFrames = 120;
constexpr float kRenderScale = 5.0f;
}  // namespace

int main(int argc, char* argv[]) {
  std::srand(static_cast<unsigned>(std::time(nullptr)));

  double dt = kDefaultDt;
  if (argc > 1) {
    dt = std::atof(argv[1]);
    if (dt <= 0.0) {
      std::printf("Invalid dt value provided. Using default dt = %.3f\n",
                  kDefaultDt);
      dt = kDefaultDt;
    }
  }
  std::printf("Using dt = %.4f seconds\n", dt);

  fsai_clock_config clock_cfg{};
  clock_cfg.mode = FSAI_CLOCK_MODE_SIMULATED;
  clock_cfg.start_time_ns = 0;
  fsai_clock_init(clock_cfg);

  const uint64_t step_ns = fsai_clock_from_seconds(dt);
  const double step_seconds = fsai_clock_to_seconds(step_ns);

  fsai_budget_init();
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_SIMULATION, "Simulation Renderer",
                        fsai_clock_from_seconds(1.0 / 60.0));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CONTROL, "Planner + Controller",
                        fsai_clock_from_seconds(0.005));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_VISION, "Vision Pipeline",
                        fsai_clock_from_seconds(0.020));
  fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CAN, "CAN Dispatch",
                        fsai_clock_from_seconds(0.002));
  fsai_budget_mark_unimplemented(
      FSAI_BUDGET_SUBSYSTEM_VISION,
      "Vision pipeline not yet integrated; timing scaffold active.");
  fsai_budget_mark_unimplemented(
      FSAI_BUDGET_SUBSYSTEM_CAN,
      "CAN transport not wired; pending hardware integration.");

  World world;
  world.init("../configs/vehicle/configDry.yaml");

  Graphics graphics{};
  if (Graphics_Init(&graphics, "Car Simulation 2D", kWindowWidth,
                    kWindowHeight) != 0) {
    std::fprintf(stderr, "Graphics_Init failed\n");
    return EXIT_FAILURE;
  }

  fsai::sim::integration::TerminalKeyboard keyboard{};
  fsai::sim::integration::CsvLogger logger("CarStateLog.csv", "RALog.csv");
  if (!logger.valid()) {
    std::fprintf(stderr, "Failed to open CSV logs\n");
    Graphics_Cleanup(&graphics);
    return EXIT_FAILURE;
  }

  fsai::sim::integration::registerBuiltInStereoProviders();
  auto stereo_factory =
      fsai::sim::integration::lookupStereoProvider("sim_stereo");
  std::unique_ptr<fsai::io::camera::sim_stereo::SimStereoSource> stereo_source;
  if (stereo_factory) {
    stereo_source = stereo_factory();
  }
  fsai::sim::integration::StereoDisplay stereo_display;

  std::vector<std::array<float, 3>> cone_positions;
  bool running = true;
  size_t frame_counter = 0;

  while (running) {
    SDL_Event event;
    while (SDL_PollEvent(&event)) {
      if (event.type == SDL_QUIT) {
        running = false;
      }
    }
    if (!running) {
      break;
    }

    const int key = keyboard.poll();
    if (key == 'q' || key == 'Q') {
      running = false;
    } else if (key == 'm' || key == 'M') {
      world.useRacingAlgorithm = world.useRacingAlgorithm ? 0 : 1;
      std::printf("Switched to %s control mode.\n",
                  world.useRacingAlgorithm ? "automatic" : "manual");
    } else if (!world.useRacingAlgorithm && key != -1) {
      if (key == 'w' || key == 'W') {
        world.throttleInput = 1.0f;
      } else if (key == 's' || key == 'S') {
        world.throttleInput = -1.0f;
      }
      if (key == 'a' || key == 'A') {
        world.steeringAngle = -1.0f;
      } else if (key == 'd' || key == 'D') {
        world.steeringAngle = 1.0f;
      }
    }

    const uint64_t now_ns = fsai_clock_advance(step_ns);
    {
      fsai::time::ControlStageTimer control_timer("world_update");
      world.update(step_seconds);
    }
    const double sim_time_s = fsai_clock_to_seconds(now_ns);

    logger.logState(sim_time_s, world.vehicleState());
    logger.logControl(sim_time_s, world.throttleInput, world.steeringAngle);

    if (stereo_source) {
      const auto& transform = world.vehicleTransform();
      stereo_source->setBodyPose(transform.position.x, transform.position.y,
                                 transform.position.z, transform.yaw);

      cone_positions.clear();
      const auto& left_cones = world.leftConePositions();
      const auto& right_cones = world.rightConePositions();
      cone_positions.reserve(left_cones.size() + right_cones.size());
      for (const auto& cone : left_cones) {
        cone_positions.push_back({cone.x, cone.y, cone.z});
      }
      for (const auto& cone : right_cones) {
        cone_positions.push_back({cone.x, cone.y, cone.z});
      }
      stereo_source->setCones(cone_positions);
      const FsaiStereoFrame& frame = stereo_source->capture(now_ns);
      stereo_display.present(frame);
    }

    {
      fsai::time::SimulationStageTimer render_timer("renderer");
      Graphics_Clear(&graphics);
      Graphics_DrawGrid(&graphics, 50);

      const auto& checkpoints = world.checkpointPositionsWorld();
      if (!checkpoints.empty()) {
        SDL_SetRenderDrawColor(graphics.renderer, 200, 0, 200, 255);
        Graphics_DrawFilledCircle(
            &graphics,
            static_cast<int>(checkpoints.front().x * kRenderScale +
                             graphics.width / 2.0f),
            static_cast<int>(checkpoints.front().z * kRenderScale +
                             graphics.height / 2.0f),
            static_cast<int>(1.5f * kRenderScale));
      }

      const auto& lookahead = world.lookahead();
      SDL_SetRenderDrawColor(graphics.renderer, 120, 120, 120, 255);
      for (size_t i = 0; i < world.leftConePositions().size(); ++i) {
        if (i == 0) {
          SDL_SetRenderDrawColor(graphics.renderer, 0, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.speed) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.steer) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 0, 255, 255);
        } else {
          SDL_SetRenderDrawColor(graphics.renderer, 120, 120, 120, 255);
        }
        const auto& cone = world.leftConePositions()[i];
        const int cone_x = static_cast<int>(cone.x * kRenderScale +
                                            graphics.width / 2.0f);
        const int cone_y = static_cast<int>(cone.z * kRenderScale +
                                            graphics.height / 2.0f);
        Graphics_DrawFilledCircle(&graphics, cone_x, cone_y,
                                  static_cast<int>(kRenderScale));
      }

      for (size_t i = 0; i < world.rightConePositions().size(); ++i) {
        if (i == 0) {
          SDL_SetRenderDrawColor(graphics.renderer, 0, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.speed) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 255, 0, 255);
        } else if (static_cast<int>(i) == lookahead.steer) {
          SDL_SetRenderDrawColor(graphics.renderer, 255, 0, 255, 255);
        } else {
          SDL_SetRenderDrawColor(graphics.renderer, 80, 80, 80, 255);
        }
        const auto& cone = world.rightConePositions()[i];
        const int cone_x = static_cast<int>(cone.x * kRenderScale +
                                            graphics.width / 2.0f);
        const int cone_y = static_cast<int>(cone.z * kRenderScale +
                                            graphics.height / 2.0f);
        Graphics_DrawFilledCircle(&graphics, cone_x, cone_y,
                                  static_cast<int>(kRenderScale));
      }

      const auto& transform = world.vehicleTransform();
      const float car_screen_x = transform.position.x * kRenderScale +
                                 graphics.width / 2.0f;
      const float car_screen_y = transform.position.z * kRenderScale +
                                 graphics.height / 2.0f;
      const float car_radius = 2.0f * kRenderScale;
      Graphics_DrawCar(&graphics, car_screen_x, car_screen_y, car_radius,
                       transform.yaw);
      Graphics_Present(&graphics);
    }

    SDL_Delay(static_cast<Uint32>(step_seconds * 1000.0));

    frame_counter++;
    if (frame_counter % kReportIntervalFrames == 0) {
      fsai_budget_report_all();
    }
  }

  fsai_budget_report_all();
  Graphics_Cleanup(&graphics);
  std::printf("Simulation complete. Car state log saved to CarStateLog.csv\n");
  return EXIT_SUCCESS;
}
