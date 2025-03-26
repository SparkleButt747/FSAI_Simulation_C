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

int main(void) {
    // Seed random number generator.
    srand((unsigned) time(NULL));

    // Initialize keyboard input.
    KeyboardInputHandler_Init();

    // Initialize the car controller with the YAML configuration file.
    CarController controller;
    CarController_Init(&controller, "configDry.yaml");

    // Open CSV file to record the car state log.
    FILE *csvFile = fopen("CarStateLog.csv", "w");
    if (!csvFile) {
        perror("Failed to open car_telemetry.csv");
        exit(EXIT_FAILURE);
    }
    // Write CSV header.
    fprintf(csvFile, "time,x,y,z,yaw,v_x,v_y,v_z\n");

    // Initialize graphics.
    Graphics g;
    if (Graphics_Init(&g, "Car Simulation", 800, 600) != 0) {
        fprintf(stderr, "Graphics_Init failed\n");
        exit(EXIT_FAILURE);
    }

    // Simulation loop: run until 'c' is pressed.
    double dt = 0.01;      // 10ms timestep
    double totalTime = 0.0;
    while (1) {
        // Poll SDL events.
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                goto exit_loop;
            }
        }
        
        // Read a key from the keyboard.
        int key = KeyboardInputHandler_GetInput();
        if (key == 'c' || key == 'C') {
            printf("Exit key 'c' detected. Exiting simulation loop.\n");
            break;
        }
        
        // Use keyboard input for control.
        carController:
        // Default: no input.
        controller.carInput.acc = 0.0;
        controller.carInput.delta = 0.0;
        if (key != -1) {
            if (key == 'w' || key == 'W') {
                controller.carInput.acc = 1.0;  // Accelerate
            } else if (key == 's' || key == 'S') {
                controller.carInput.acc = -1.0; // Decelerate
            }
            if (key == 'a' || key == 'A') {
                controller.carInput.delta = -1.0; // Full left
            } else if (key == 'd' || key == 'D') {
                controller.carInput.delta = 1.0;  // Full right
            }
        }
        
        // Update the car's state using the dynamic bicycle model.
        DynamicBicycle_UpdateState(&controller.carModel, &controller.carState, &controller.carInput, dt);
        
        // Log state to CSV.
        fprintf(csvFile, "%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                totalTime, controller.carState.x, controller.carState.y, controller.carState.z,
                controller.carState.yaw, controller.carState.v_x, controller.carState.v_y, controller.carState.v_z);
        
        totalTime += dt;
        
        // --- Graphics Update ---
        Graphics_Clear(&g);
        Graphics_DrawGrid(&g, 50); // Draw grid with 50-pixel spacing.
        
        // Draw cones as black circles.
        SDL_SetRenderDrawColor(g.renderer, 0, 0, 0, 255);  // Black color.
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
        
        // Draw the car in red.
        SDL_SetRenderDrawColor(g.renderer, 0, 0, 255, 255);  // Red color.
        int carX = (int)(controller.carState.x + g.width/2);
        int carY = (int)(controller.carState.y + g.height/2);
        Graphics_DrawCar(&g, carX, carY, 10, controller.carState.yaw);
        
        Graphics_Present(&g);
        
        SDL_Delay((Uint32)(dt * 1000));
    }

exit_loop:
    fclose(csvFile);
    KeyboardInputHandler_Restore();
    Graphics_Cleanup(&g);
    
    printf("Simulation complete. Car state log saved to CarStateLog.csv\n");
    return EXIT_SUCCESS;
}
