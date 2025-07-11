#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include "DynamicBicycle.h"
#include "State.h"
#include "Input.h"
#include "KeyboardInputHandler.h"  // Keyboard input handler
#include "Graphics.h"              // Graphics module

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void) {
    // Seed random number generator.
    srand((unsigned) time(NULL));

    // Create instances of the dynamic bicycle model, state, and input.
    DynamicBicycle carModel;
    State carState;
    Input carInput;
    
    // Simulation time settings.
    double totalTime = 0.0;
    double dt = 0.05;  // 10ms timestep
        
    // Initialize the dynamic bicycle model.
    DynamicBicycle_init(&carModel, "configDry.yaml");
    
    // Initialize input and state.
    carInput = Input_create(0.0, 0.0, 0.0);
    carState = State_create(0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    
    // Open CSV file to record the car state log.
    FILE* csvFile = fopen("CarStateLog.csv", "w");
    if (!csvFile) {
        fprintf(stderr, "Error opening file for writing.\n");
        return EXIT_FAILURE;
    }
    // Write CSV header.
    fprintf(csvFile, "Time,x,y,z,yaw,v_x,v_y,v_z,r_x,r_y,r_z,a_x,a_y,a_z\n");

    // Initialize graphics.
    Graphics g;
    if (Graphics_Init(&g, "Car Simulation", 800, 600) != 0) {
        fprintf(stderr, "Graphics_Init failed\n");
        exit(EXIT_FAILURE);
    }

    // — test parameters (tweak these as you like) —
    const double testDuration     = 10.0;   // seconds
    const double logInterval      =  0.1;   // seconds
    const double thAmp            =  1.0;   // throttle amplitude
    const double thFreq           =  0.5;   // Hz
    const double thPhase          =  0.0;   // radians
    const double stAmp            =  1.0;   // steering amplitude
    const double stFreq           =  0.2;   // Hz
    const double stPhase          =  0.0;   // radians

    double nextLog    = 0.0;
    
    // Simulation loop: run until 'c' is pressed.
    while (1) {
        // Poll SDL events to keep window responsive.
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            if (event.type == SDL_QUIT) {
                printf("SDL_QUIT event received. Exiting simulation loop.\n");
                goto exit_loop;
            }
        }

        // exit the loop.
        if (totalTime < testDuration) {
            printf("Exit key 'c' detected. Exiting simulation loop.\n");
            break;
        }
 
        carInput.acc   = thAmp * sin(2.0*M_PI*thFreq*totalTime + thPhase);
        carInput.delta = stAmp * sin(2.0*M_PI*stFreq*totalTime + stPhase);
        
        // Update the car's state.
        DynamicBicycle_UpdateState(&carModel, &carState, &carInput, dt);
        
        // 3) log at fixed intervals
        if (totalTime >= nextLog) {
            fprintf(csvFile,
              "%.2f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
              totalTime,
              carState.x, carState.y, carState.z, carState.yaw,
              carState.v_x, carState.v_y, carState.v_z,
              carState.r_x, carState.r_y, carState.r_z,
              carState.a_x, carState.a_y, carState.a_z
            );
            nextLog += logInterval;
        }
        
        // --- Graphics Update ---
        Graphics_Clear(&g);
        Graphics_DrawGrid(&g, 50);  // Draw grid with spacing of 50 pixels.
        // Here, adjust mapping as necessary. This example assumes simulation coordinates map directly.
        Graphics_DrawCar(&g, (float)carState.x, (float)carState.y, 10, (float)carState.yaw);
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
