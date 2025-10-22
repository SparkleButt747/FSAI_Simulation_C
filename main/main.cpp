#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include "fsai_clock.h"
#include "types.h"
#include "CarController.hpp"
#include "Graphics.h"
#include "KeyboardInputHandler.h"
#include "budget.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

#define SCALE 5

int main(int argc, char* argv[]) {
    // Seed random number generator.
    srand((unsigned) time(NULL));

    // Parse dt from command line if provided; default to 0.01 seconds.
    double dt = 0.01;
    if (argc > 1) {
        dt = std::atof(argv[1]);
        if (dt <= 0) {
            std::printf("Invalid dt value provided. Using default dt = 0.01\n");
            dt = 0.01;
        }
    }
    std::printf("Using dt = %.4f seconds\n", dt);

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
                                   "Vision pipeline not yet integrated; timing scaffold active.");
    fsai_budget_mark_unimplemented(FSAI_BUDGET_SUBSYSTEM_CAN,
                                   "CAN transport not wired; pending hardware integration.");

    // Initialize the car controller.
    CarController controller;
    CarController_Init(&controller, "../configs/vehicle/configDry.yaml");
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
        {
            fsai::time::ControlStageTimer control_timer("control_tick");
            CarController_Update(&controller, step_seconds, now_ns);
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
            Graphics_DrawGrid(&g, 50);  // Draw grid with 50-pixel spacing.
            SDL_SetRenderDrawColor(g.renderer, 200, 0, 200, 255);
            Graphics_DrawFilledCircle(&g, controller.checkpointPositions[0].x*SCALE + g.width/2, controller.checkpointPositions[0].z*SCALE + g.height/2, 1.5*SCALE);

            // Draw cones as black circles.
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
                int coneX = (int)(controller.leftCones[i].x*SCALE + g.width/2);
                int coneY = (int)(controller.leftCones[i].z*SCALE + g.height/2);
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
                int coneX = (int)(controller.rightCones[i].x*SCALE + g.width/2);
                int coneY = (int)(controller.rightCones[i].z*SCALE + g.height/2);
                Graphics_DrawFilledCircle(&g, coneX, coneY, SCALE);
            }

            // Map simulation coordinates to screen coordinates.
            float carScreenX = static_cast<float>(controller.carState.position.x())*SCALE + g.width / 2.0f;
            float carScreenY = static_cast<float>(controller.carState.position.y())*SCALE + g.height / 2.0f;
            float carRadius = 2.0f*SCALE;
            float carYaw = static_cast<float>(controller.carState.yaw);
            // Draw the car using Graphics_DrawCar.
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
