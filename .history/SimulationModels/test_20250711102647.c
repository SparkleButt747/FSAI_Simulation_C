#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>
#include <SDL2/SDL.h>           // for SDL_Delay
#include "DynamicBicycle.h"
#include "State.h"
#include "Input.h"
#include "KeyboardInputHandler.h"
#include "Graphics.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

int main(void) {
    // — seed RNG (not really used by test) —
    srand((unsigned)time(NULL));

    // — init model, input, state —
    DynamicBicycle  carModel;
    State           carState;
    Input           carInput;

    DynamicBicycle_init(&carModel, "configDry.yaml");
    carInput = Input_create(0,0,0);
    carState = State_create(0,0,0, 0,0,0, 0,0,0, 0,0,0, 0);

    // — open CSV log —
    FILE *csvFile = fopen("CarStateLog.csv", "w");
    if (!csvFile) {
        fprintf(stderr, "Error opening CarStateLog.csv for write\n");
        return EXIT_FAILURE;
    }
    fprintf(csvFile,
      "time,x,y,z,yaw,v_x,v_y,v_z,r_x,r_y,r_z,a_x,a_y,a_z\n"
    );

    // — init graphics (optional) —
    Graphics g;
    if (Graphics_Init(&g, "Car Simulation Test", 800,600) != 0) {
        fprintf(stderr, "Graphics_Init failed\n");
        fclose(csvFile);
        return EXIT_FAILURE;
    }

    // — test parameters (tweak these as you like) —
    const double testDuration     = 10.0;   // seconds
    const double logInterval      =  0.1;   // seconds
    const double dt               =  0.05;  // timestep, matches your original
    const double thAmp            =  1.0;   // throttle amplitude
    const double thFreq           =  0.5;   // Hz
    const double thPhase          =  0.0;   // radians
    const double stAmp            =  1.0;   // steering amplitude
    const double stFreq           =  0.2;   // Hz
    const double stPhase          =  0.0;   // radians

    double totalTime = 0.0;
    double nextLog    = 0.0;

    // — test loop —
    while (totalTime < testDuration) {
        // 1) generate sine-wave inputs
        carInput.acc   = thAmp * sin(2.0*M_PI*thFreq*totalTime + thPhase);
        carInput.delta = stAmp * sin(2.0*M_PI*stFreq*totalTime + stPhase);

        // 2) advance the model
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

        // 4) render (optional)
        Graphics_Clear(&g);
        Graphics_DrawGrid(&g, 50);
        Graphics_DrawCar(&g,
          (float)carState.x,
          (float)carState.y,
          10,
          (float)carState.yaw
        );
        Graphics_Present(&g);

        // 5) wait and advance time
        SDL_Delay((Uint32)(dt*1000));
        totalTime += dt;
    }

    // — cleanup —
    fclose(csvFile);
    KeyboardInputHandler_Restore();
    Graphics_Cleanup(&g);

    printf("Test complete. Logged %.1f s of data to CarStateLog.csv\n", testDuration);
    return EXIT_SUCCESS;
}
