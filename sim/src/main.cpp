#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <exception>
#include <string>
#include <vector>

#include "fsai_clock.h"
#include "common/types.h"
#include "control/clc_controller.hpp"
#include "Graphics.h"
#include "sim/KeyboardInputHandler.h"
#include "budget.h"
#include "PathConfig.hpp"
#include "sim/integration/adapter.hpp"
#include "sim/integration/fake_providers.hpp"
#include "sim/integration/path_truth.hpp"
#include "sim/integration/provider_registry.hpp"
#include "sim/integration/runtime_config.hpp"

#include "Vector.h"

namespace integration = fsai::integration;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SCALE 3

int main(int argc, char* argv[]) {
    // Seed random number generator.
    srand((unsigned) time(NULL));

    // Parse runtime parameters.
    double dt = 0.01;
    std::string config_path = "../config/runtime.yaml";
    for (int i = 1; i < argc; ++i) {
        std::string arg = argv[i];
        if (arg.rfind("--config=", 0) == 0) {
            config_path = arg.substr(9);
        } else if (arg.rfind("--dt=", 0) == 0) {
            dt = std::atof(arg.substr(5).c_str());
        } else if (!arg.empty() && arg[0] != '-') {
            dt = std::atof(arg.c_str());
        }
    }
    if (dt <= 0.0) {
        std::printf("Invalid dt value provided. Using default dt = 0.01\n");
        dt = 0.01;
    }
    std::printf("Using dt = %.4f seconds (config: %s)\n", dt, config_path.c_str());

    // Initialize keyboard input.
    //KeyboardInputHandler_Init();

    fsai_clock_config clock_cfg{};
    clock_cfg.mode = FSAI_CLOCK_MODE_SIMULATED;
    clock_cfg.start_time_ns = 0;
    fsai_clock_init(clock_cfg);

    const uint64_t step_ns = fsai_clock_from_seconds(dt);
    const double step_seconds = fsai_clock_to_seconds(step_ns);

    fsai_budget_init();
    // Budgets derived from the FS-AI v1.1 contract: render @60 Hz (~16.6 ms),
    // vision ≤20 ms, planner+controller sub-5 ms, CAN within ~2 ms of stamping.
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_SIMULATION, "Simulation Renderer", fsai_clock_from_seconds(1.0 / 60.0));
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CONTROL, "Planner + Controller", fsai_clock_from_seconds(0.005));
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_VISION, "Vision Pipeline", fsai_clock_from_seconds(0.020));
    fsai_budget_configure(FSAI_BUDGET_SUBSYSTEM_CAN, "CAN Dispatch", fsai_clock_from_seconds(0.002));
    fsai_budget_mark_unimplemented(FSAI_BUDGET_SUBSYSTEM_VISION,
                                   "Using fake vision provider; timing scaffold active.");
    fsai_budget_mark_unimplemented(FSAI_BUDGET_SUBSYSTEM_CAN,
                                   "Using fake CAN provider; timing scaffold active.");

    integration::RuntimeConfig runtime_cfg;
    try {
        runtime_cfg = integration::LoadRuntimeConfig(config_path);
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "Failed to load runtime config: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    PathConfig path_config;
    integration::PathTruth path_truth = integration::GeneratePathTruth(path_config);

    integration::ProviderRegistry registry;
    registry.RegisterVision(integration::MakeFakeVisionProvider(path_truth, runtime_cfg.fake_vision));
    registry.RegisterPlanner(integration::MakeFakePlannerProvider(path_truth, runtime_cfg.fake_planner));
    registry.RegisterEstimator(integration::MakeFakeEstimatorProvider());
    registry.RegisterCan(integration::MakeFakeCanProvider());

    integration::IVisionProvider* vision = nullptr;
    integration::IPlannerProvider* planner = nullptr;
    integration::IEstimatorProvider* estimator = nullptr;
    integration::ICanProvider* can = nullptr;
    try {
        vision = registry.ActivateVision(runtime_cfg.providers.vision);
        planner = registry.ActivatePlanner(runtime_cfg.providers.planner);
        estimator = registry.ActivateEstimator(runtime_cfg.providers.estimator);
        can = registry.ActivateCan(runtime_cfg.providers.can);
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "Provider activation failed: %s\n", ex.what());
        return EXIT_FAILURE;
    }

    // Initialize the car controller.
    CarController controller;
    CarController_Init(&controller, "../sim/src/vehicle/Configs/configDry.yaml", &path_truth);

    std::vector<Vector3> checkpoint_buffer(1024);
    FsaiDetections detections{};
    // Open CSV file to record car state log.
    FILE* csvFile = fopen("CarStateLog.csv", "w");
    if (!csvFile) {
        std::perror("Failed to open CarStateLog.csv");
        std::exit(EXIT_FAILURE);
    }
    std::fprintf(csvFile, "time,x,y,z,yaw,v_x,v_y,v_z\n");

    // Open CSV file to record car state log.
    FILE* csvFile_ra = fopen("RALog.csv", "w");
    if (!csvFile_ra) {
        std::perror("Failed to open RALog.csv");
        std::exit(EXIT_FAILURE);
    }
    std::fprintf(csvFile_ra, "time,t,s\n");

    // Initialize graphics.
    Graphics g;
    if (Graphics_Init(&g, "Car Simulation", 800, 600) != 0) {
        std::fprintf(stderr, "Graphics_Init failed\n");
        std::exit(EXIT_FAILURE);
    }
    size_t frame_counter = 0;
    while (1) {
        // Poll SDL events to keep window responsive.
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                goto exit_loop;
            }
        }

        /*
        // Read keyboard input.
        int key = KeyboardInputHandler_GetInput();
        // If 'c' or 'C' is pressed, exit the simulation loop.
        if (key == 'c' || key == 'C') {
            std::printf("Exit key 'c' detected. Exiting simulation loop.\n");
            break;
        }
        */

        const uint64_t now_ns = fsai_clock_advance(step_ns);

        if (vision) {
            detections.n = 0;
            vision->Process(nullptr, &detections);
        }

        integration::PathMeta path_meta;
        if (planner) {
            path_meta = planner->BuildPathMeta(controller.carState);
        }

        if (estimator) {
            (void)estimator->Estimate(controller.carState);
        }

        integration::AdapterOutput adapter_output = integration::Adapter_BuildCheckpoints(
            detections.n > 0 ? &detections : nullptr,
            planner ? &path_meta : nullptr,
            checkpoint_buffer.data(),
            static_cast<int>(checkpoint_buffer.size()),
            runtime_cfg.fake_planner.sample_spacing_m,
            runtime_cfg.fake_planner.horizon_m);

        if (adapter_output.count > 0) {
            CarController_SetCheckpoints(&controller, checkpoint_buffer.data(), adapter_output.count);
        } else {
            CarController_SetCheckpoints(&controller, nullptr, 0);
        }

        FsaiControlCmd cmd{};
        {
            fsai::time::ControlStageTimer control_timer("control_tick");
            cmd = CarController_Update(&controller, step_seconds, now_ns);
        }

        if (can) {
            can->Send(cmd);
            FsaiCanMsg hb{};
            while (can->Poll(&hb)) {
            }
        }

        const double sim_time_s = fsai_clock_to_seconds(now_ns - controller.startTimeNs);

        // Log current state to CSV.
        std::fprintf(csvFile, "%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                     sim_time_s, controller.carState.position.x(), controller.carState.position.y(),
                     controller.carState.position.z(), controller.carState.yaw,
                     controller.carState.velocity.x(), controller.carState.velocity.y(), controller.carState.velocity.z());

        // Log current state to CSV.
        std::fprintf(csvFile_ra, "%.6f,%.6f,%.6f\n",
                     sim_time_s, controller.throttleInput, controller.steeringAngle);

        // --- Graphics Update ---
        {
            fsai::time::SimulationStageTimer render_timer("renderer");
            Graphics_Clear(&g);
            Graphics_DrawGrid(&g, 50);

            if (controller.checkpointPositions && controller.nCheckpoints > 0) {
                SDL_SetRenderDrawColor(g.renderer, 200, 0, 200, 255);
                const Vector3& next = controller.checkpointPositions[0];
                Graphics_DrawFilledCircle(&g,
                                          next.x * SCALE + g.width / 2,
                                          next.z * SCALE + g.height / 2,
                                          1.5f * SCALE);
            }

            for (const Transform& cone : path_truth.track.leftCones) {
                SDL_SetRenderDrawColor(g.renderer, 80, 80, 80, 255);
                int coneX = static_cast<int>(cone.position.x * SCALE + g.width / 2);
                int coneY = static_cast<int>(cone.position.z * SCALE + g.height / 2);
                Graphics_DrawFilledCircle(&g, coneX, coneY, SCALE);
            }
            for (const Transform& cone : path_truth.track.rightCones) {
                SDL_SetRenderDrawColor(g.renderer, 80, 80, 80, 255);
                int coneX = static_cast<int>(cone.position.x * SCALE + g.width / 2);
                int coneY = static_cast<int>(cone.position.z * SCALE + g.height / 2);
                Graphics_DrawFilledCircle(&g, coneX, coneY, SCALE);
            }

            if (controller.checkpointPositions && controller.nCheckpoints > 0) {
                SDL_SetRenderDrawColor(g.renderer, 0, 200, 255, 255);
                for (int i = 0; i < controller.nCheckpoints; ++i) {
                    const Vector3& cp = controller.checkpointPositions[i];
                    Graphics_DrawFilledCircle(&g,
                                              cp.x * SCALE + g.width / 2,
                                              cp.z * SCALE + g.height / 2,
                                              SCALE / 2);
                }
            }

            float carScreenX = static_cast<float>(controller.carState.position.x()) * SCALE + g.width / 2.0f;
            float carScreenY = static_cast<float>(controller.carState.position.y()) * SCALE + g.height / 2.0f;
            float carRadius = 2.0f * SCALE;
            float carYaw = static_cast<float>(controller.carState.yaw);
            Graphics_DrawCar(&g, carScreenX, carScreenY, carRadius, carYaw);
            Graphics_Present(&g);
        }

        SDL_Delay((Uint32)(step_seconds * 1000.0));

        frame_counter++;
        if (frame_counter % 120 == 0) {
            fsai_budget_report_all();
        }
    }

exit_loop:
    fclose(csvFile);
    fclose(csvFile_ra);
    KeyboardInputHandler_Restore();
    Graphics_Cleanup(&g);

    fsai_budget_report_all();

    std::printf("Simulation complete. Car state log saved to CarStateLog.csv\n");
    return EXIT_SUCCESS;
}
