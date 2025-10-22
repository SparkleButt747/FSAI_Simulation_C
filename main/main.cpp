#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <vector>
#include <array>

#include <SDL.h>

#include "CarController.hpp"
#include "Graphics.h"
#include "KeyboardInputHandler.h"
#include "budget.h"
#include "fsai_clock.h"
#include "sim_stereo_source.hpp"
#include "types.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SCALE 5

using fsai::io::camera::sim_stereo::SimStereoConfig;
using fsai::io::camera::sim_stereo::SimStereoSource;

int main(int argc, char* argv[]) {
    srand((unsigned) time(NULL));

    double dt = 0.01;
    if (argc > 1) {
        dt = std::atof(argv[1]);
        if (dt <= 0) {
            std::printf("Invalid dt value provided. Using default dt = 0.01\n");
            dt = 0.01;
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
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_SIMULATION, "Simulation Renderer", fsai_clock_from_seconds(1.0 / 60.0));
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CONTROL, "Planner + Controller", fsai_clock_from_seconds(0.005));
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_VISION, "Vision Pipeline", fsai_clock_from_seconds(0.020));
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CAN, "CAN Dispatch", fsai_clock_from_seconds(0.002));
    fsai_budget_mark_unimplemented(FSAI_BUDGET_SUBSYSTEM_VISION,
                                   "Vision pipeline not yet integrated; timing scaffold active.");
    fsai_budget_mark_unimplemented(FSAI_BUDGET_SUBSYSTEM_CAN,
                                   "CAN transport not wired; pending hardware integration.");

    CarController controller;
    CarController_Init(&controller, "../configs/vehicle/configDry.yaml");

    FILE* csvFile = fopen("CarStateLog.csv", "w");
    if (!csvFile) {
        std::perror("Failed to open CarStateLog.csv");
        return EXIT_FAILURE;
    }
    std::fprintf(csvFile, "time,x,y,z,yaw,v_x,v_y,v_z\n");

    FILE* csvFile_ra = fopen("RALog.csv", "w");
    if (!csvFile_ra) {
        std::perror("Failed to open RALog.csv");
        fclose(csvFile);
        return EXIT_FAILURE;
    }
    std::fprintf(csvFile_ra, "time,t,s\n");

    Graphics g;
    if (Graphics_Init(&g, "Car Simulation 2D", 800, 600) != 0) {
        std::fprintf(stderr, "Graphics_Init failed\n");
        fclose(csvFile);
        fclose(csvFile_ra);
        return EXIT_FAILURE;
    }

    SimStereoConfig stereo_cfg;
    stereo_cfg.width = 640;
    stereo_cfg.height = 480;
    stereo_cfg.near_plane = 0.1f;
    stereo_cfg.far_plane = 200.0f;
    stereo_cfg.noise_stddev = 0.0f;
    stereo_cfg.intrinsics.fx = 620.0f;
    stereo_cfg.intrinsics.fy = 620.0f;
    stereo_cfg.intrinsics.cx = stereo_cfg.width / 2.0f;
    stereo_cfg.intrinsics.cy = stereo_cfg.height / 2.0f;
    for (int i = 0; i < 9; ++i) {
        stereo_cfg.left_extrinsics.R[i] = 0.0f;
        stereo_cfg.right_extrinsics.R[i] = 0.0f;
    }
    stereo_cfg.left_extrinsics.R[0] = stereo_cfg.left_extrinsics.R[4] = stereo_cfg.left_extrinsics.R[8] = 1.0f;
    stereo_cfg.right_extrinsics.R[0] = stereo_cfg.right_extrinsics.R[4] = stereo_cfg.right_extrinsics.R[8] = 1.0f;
    stereo_cfg.left_extrinsics.t[0] = 0.4f;
    stereo_cfg.left_extrinsics.t[1] = 0.25f;
    stereo_cfg.left_extrinsics.t[2] = 0.12f;
    stereo_cfg.right_extrinsics.t[0] = 0.4f;
    stereo_cfg.right_extrinsics.t[1] = 0.25f;
    stereo_cfg.right_extrinsics.t[2] = -0.12f;

    int exit_code = EXIT_SUCCESS;

    {
        SimStereoSource stereo(stereo_cfg);

        SDL_Window* stereo_window = SDL_CreateWindow(
            "FS-AI Stereo Simulation", SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
            stereo_cfg.width * 2, stereo_cfg.height, SDL_WINDOW_SHOWN);
        SDL_Renderer* stereo_renderer = nullptr;
        SDL_Texture* left_texture = nullptr;
        SDL_Texture* right_texture = nullptr;

        if (!stereo_window) {
            std::fprintf(stderr, "Failed to create stereo window: %s\n", SDL_GetError());
            exit_code = EXIT_FAILURE;
        } else {
            stereo_renderer = SDL_CreateRenderer(
                stereo_window, -1,
                SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
            if (!stereo_renderer) {
                std::fprintf(stderr, "Failed to create stereo renderer: %s\n",
                             SDL_GetError());
                exit_code = EXIT_FAILURE;
            } else {
                left_texture = SDL_CreateTexture(stereo_renderer, SDL_PIXELFORMAT_RGB24,
                                                 SDL_TEXTUREACCESS_STREAMING,
                                                 stereo_cfg.width, stereo_cfg.height);
                right_texture = SDL_CreateTexture(stereo_renderer, SDL_PIXELFORMAT_RGB24,
                                                  SDL_TEXTUREACCESS_STREAMING,
                                                  stereo_cfg.width, stereo_cfg.height);
                if (!left_texture || !right_texture) {
                    std::fprintf(stderr,
                                 "Failed to create stereo textures: %s\n",
                                 SDL_GetError());
                    exit_code = EXIT_FAILURE;
                }
            }
        }

        std::vector<std::array<float, 3>> cone_positions;
        size_t frame_counter = 0;

        if (exit_code == EXIT_SUCCESS) {
            bool running = true;
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

                const uint64_t now_ns = fsai_clock_advance(step_ns);
                {
                    fsai::time::ControlStageTimer control_timer("control_tick");
                    CarController_Update(&controller, step_seconds, now_ns);
                }
                const double sim_time_s =
                    fsai_clock_to_seconds(now_ns - controller.startTimeNs);

                std::fprintf(csvFile,
                             "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                             sim_time_s, controller.carState.position.x(),
                             controller.carState.position.y(),
                             controller.carState.position.z(),
                             controller.carState.yaw,
                             controller.carState.velocity.x(),
                             controller.carState.velocity.y(),
                             controller.carState.velocity.z());

                std::fprintf(csvFile_ra, "%.6f,%.6f,%.6f\n", sim_time_s,
                             controller.throttleInput, controller.steeringAngle);

                stereo.setBodyPose(static_cast<float>(controller.carTransform.position.x),
                                   static_cast<float>(controller.carTransform.position.y),
                                   static_cast<float>(controller.carTransform.position.z),
                                   static_cast<float>(controller.carTransform.yaw));

                cone_positions.clear();
                cone_positions.reserve(static_cast<size_t>(controller.nLeftCones +
                                                           controller.nRightCones));
                for (int i = 0; i < controller.nLeftCones; i++) {
                    cone_positions.push_back({controller.leftCones[i].x, 0.0f,
                                              controller.leftCones[i].z});
                }
                for (int i = 0; i < controller.nRightCones; i++) {
                    cone_positions.push_back({controller.rightCones[i].x, 0.0f,
                                              controller.rightCones[i].z});
                }
                stereo.setCones(cone_positions);

                const FsaiStereoFrame& frame = stereo.capture(now_ns);

                SDL_UpdateTexture(left_texture, nullptr, frame.left.data,
                                  frame.left.stride);
                SDL_UpdateTexture(right_texture, nullptr, frame.right.data,
                                  frame.right.stride);

                SDL_RenderClear(stereo_renderer);
                SDL_Rect left_rect{0, 0, stereo_cfg.width, stereo_cfg.height};
                SDL_Rect right_rect{stereo_cfg.width, 0, stereo_cfg.width,
                                    stereo_cfg.height};
                SDL_RenderCopy(stereo_renderer, left_texture, nullptr, &left_rect);
                SDL_RenderCopy(stereo_renderer, right_texture, nullptr, &right_rect);
                SDL_RenderPresent(stereo_renderer);

                {
                    fsai::time::SimulationStageTimer render_timer("renderer");
                    Graphics_Clear(&g);
                    Graphics_DrawGrid(&g, 50);

                    SDL_SetRenderDrawColor(g.renderer, 200, 0, 200, 255);
                    Graphics_DrawFilledCircle(&g,
                                             controller.checkpointPositions[0].x * SCALE +
                                                 g.width / 2,
                                             controller.checkpointPositions[0].z * SCALE +
                                                 g.height / 2,
                                             1.5 * SCALE);

                    for (int i = 0; i < controller.nLeftCones; i++) {
                        if (i == 0) {
                            SDL_SetRenderDrawColor(g.renderer, 0, 255, 0, 255);
                        } else if (i == controller.lookaheadIndices.speed) {
                            SDL_SetRenderDrawColor(g.renderer, 255, 255, 0, 255);
                        } else if (i == controller.lookaheadIndices.steer) {
                            SDL_SetRenderDrawColor(g.renderer, 255, 0, 255, 255);
                        } else {
                            SDL_SetRenderDrawColor(g.renderer, 120, 120, 120, 255);
                        }
                        int coneX =
                            static_cast<int>(controller.leftCones[i].x * SCALE +
                                             g.width / 2);
                        int coneY =
                            static_cast<int>(controller.leftCones[i].z * SCALE +
                                             g.height / 2);
                        Graphics_DrawFilledCircle(&g, coneX, coneY, SCALE);
                    }
                    for (int i = 0; i < controller.nRightCones; i++) {
                        if (i == 0) {
                            SDL_SetRenderDrawColor(g.renderer, 0, 255, 0, 255);
                        } else if (i == controller.lookaheadIndices.speed) {
                            SDL_SetRenderDrawColor(g.renderer, 255, 255, 0, 255);
                        } else if (i == controller.lookaheadIndices.steer) {
                            SDL_SetRenderDrawColor(g.renderer, 255, 0, 255, 255);
                        } else {
                            SDL_SetRenderDrawColor(g.renderer, 80, 80, 80, 255);
                        }
                        int coneX =
                            static_cast<int>(controller.rightCones[i].x * SCALE +
                                             g.width / 2);
                        int coneY =
                            static_cast<int>(controller.rightCones[i].z * SCALE +
                                             g.height / 2);
                        Graphics_DrawFilledCircle(&g, coneX, coneY, SCALE);
                    }

                    float carScreenX =
                        static_cast<float>(controller.carState.position.x()) * SCALE +
                        g.width / 2.0f;
                    float carScreenY =
                        static_cast<float>(controller.carState.position.y()) * SCALE +
                        g.height / 2.0f;
                    float carRadius = 2.0f * SCALE;
                    float carYaw = static_cast<float>(controller.carState.yaw);
                    Graphics_DrawCar(&g, carScreenX, carScreenY, carRadius, carYaw);
                    Graphics_Present(&g);
                }

                SDL_Delay(static_cast<Uint32>(step_seconds * 1000.0));

                frame_counter++;
                if (frame_counter % 120 == 0) {
                    fsai_budget_report_all();
                }
            }
        }

        if (left_texture) {
            SDL_DestroyTexture(left_texture);
        }
        if (right_texture) {
            SDL_DestroyTexture(right_texture);
        }
        if (stereo_renderer) {
            SDL_DestroyRenderer(stereo_renderer);
        }
        if (stereo_window) {
            SDL_DestroyWindow(stereo_window);
        }
    }

    fclose(csvFile);
    fclose(csvFile_ra);
    KeyboardInputHandler_Restore();

    Graphics_Cleanup(&g);

    fsai_budget_report_all();

    std::printf("Simulation complete. Car state log saved to CarStateLog.csv\n");
    return exit_code;
}
