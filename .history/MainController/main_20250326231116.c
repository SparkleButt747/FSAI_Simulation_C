#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "CarController.h"
#include "KeyboardInputHandler.h"
#include "Graphics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(int argc, char* argv[]) {
    // Seed random number generator.
    srand((unsigned) time(NULL));

    // Parse dt from command line if provided; default to 0.01 seconds.
    double dt = 0.01;
    if (argc > 1) {
        dt = atof(argv[1]);
        if (dt <= 0) {
            printf("Invalid dt value provided. Using default dt = 0.01\n");
            dt = 0.01;
        }
    }
    printf("Using dt = %.4f seconds\n", dt);

    // Initialize keyboard input.
    KeyboardInputHandler_Init();

    // Initialize the car controller.
    CarController controller;
    CarController_Init(&controller, "configDry.yaml");

    // Open CSV file to record car state log.
    FILE* csvFile = fopen("CarStateLog.csv", "w");
    if (!csvFile) {
        perror("Failed to open CarStateLog.csv");
        exit(EXIT_FAILURE);
    }
    fprintf(csvFile, "time,x,y,z,yaw,v_x,v_y,v_z\n");

    // Open CSV file to record car state log.
    FILE* csvFile_ra = fopen("RALog.csv", "w");
    if (!csvFile) {
        perror("Failed to open RALog.csv");
        exit(EXIT_FAILURE);
    }
    fprintf(csvFile_ra, "time,t,s\n");

    // Initialize graphics.
    Graphics g;
    if (Graphics_Init(&g, "Car Simulation", 800, 600) != 0) {
        fprintf(stderr, "Graphics_Init failed\n");
        exit(EXIT_FAILURE);
    }

    double totalTime = 0.0;
    while (1) {
        // Poll SDL events to keep window responsive.
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                goto exit_loop;
            }
        }
        
        // Read keyboard input.
        int key = KeyboardInputHandler_GetInput();
        // If 'c' or 'C' is pressed, exit the simulation loop.
        if (key == 'c' || key == 'C') {
            printf("Exit key 'c' detected. Exiting simulation loop.\n");
            break;
        }
        
        // Update the car's state.
        DynamicBicycle_UpdateState(&controller.carModel, &controller.carState, &controller.carInput, dt);
        
        // Log current state to CSV.
        fprintf(csvFile, "%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                totalTime, controller.carState.x, controller.carState.y,
                controller.carState.z, controller.carState.yaw,
                controller.carState.v_x, controller.carState.v_y, controller.carState.v_z);
                    
        // Log current state to CSV.
        fprintf(csvFile_ra, "%.2f,%.6f,%.6f\n",
        totalTime, controller.throttleInput, controller.steeringAngle);

        totalTime += dt;
        
        // --- Graphics Update ---
        Graphics_Clear(&g);
        Graphics_DrawGrid(&g, 50);  // Draw grid with 50-pixel spacing.
        
        // Draw cones as black circles.
        SDL_SetRenderDrawColor(g.renderer, 0, 0, 0, 255);
        for (int i = 0; i < controller.nLeftCones; i++) {
            int coneX = (int)(controller.leftCones[i].x + g.width/2);
            int coneY = (int)(controller.leftCones[i].z + g.height/2);
            Graphics_DrawFilledCircle(&g, coneX, coneY, 5);
        }
        for (int i = 0; i < controller.nRightCones; i++) {
            int coneX = (int)(controller.rightCones[i].x + g.width/2);
            int coneY = (int)(controller.rightCones[i].z + g.height/2);
            Graphics_DrawFilledCircle(&g, coneX, coneY, 5);
        }
        
        // Map simulation coordinates to screen coordinates.
        float carScreenX = (float)controller.carState.x + g.width / 2.0f;
        float carScreenY = (float)controller.carState.y + g.height / 2.0f;
        float carRadius = 10.0f;
        float carYaw = (float)controller.carState.yaw;
        // Draw the car using Graphics_DrawCar.
        Graphics_DrawCar(&g, carScreenX, carScreenY, carRadius, carYaw);
        Graphics_Present(&g);
        
        SDL_Delay((Uint32)(dt * 1000));
    }

exit_loop:
    fclose(csvFile);
    fclose(csvFile_ra);
    KeyboardInputHandler_Restore();
    Graphics_Cleanup(&g);
    
    printf("Simulation complete. Car state log saved to CarStateLog.csv\n");
    return EXIT_SUCCESS;
}
