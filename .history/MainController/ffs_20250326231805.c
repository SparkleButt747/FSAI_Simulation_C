#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "CarController.h"
#include "KeyboardInputHandler.h" // new include for keyboard input handling

int main(void) {
    // Seed random number generator.
    srand((unsigned) time(NULL));

    // Initialize keyboard input.
    KeyboardInputHandler_Init();

    // Initialize the car controller with the YAML configuration file.
    // (CarController_Init is assumed to store the track that the car will drive in,
    // including left cones, right cones, and checkpoints.)
    CarController controller;
    CarController_Init(&controller, "configDry.yaml");

    // Open CSV file for car telemetry.
    FILE *carFile = fopen("car_telemetry.csv", "w");
    if (!carFile) {
        perror("Failed to open car_telemetry.csv");
        exit(EXIT_FAILURE);
    }
    // Write header.
    fprintf(carFile, "time,x,y,z,yaw,v_x,v_y,v_z\n");

    // Run the simulation loop.
    double dt = 0.01;              // Fixed timestep (10ms)
    double simulationTime = 5000.0;  // Total simulation time (adjust as needed)
    double elapsed = 0.0;
    while (elapsed < simulationTime) {
        CarController_Update(&controller, dt);
        elapsed += dt;
        // Record car state.
        // Note: We assume state.x and state.y represent horizontal positions.
        // Here, we map state.y to the z-coordinate.
        fprintf(carFile, "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%.2f\n",
                elapsed,
                controller.carState.x,
                controller.carState.y,
                controller.carState.z,
                controller.carState.yaw,
                controller.carState.v_x,
                controller.carState.v_y,
                controller.carState.v_z);
    }
    fclose(carFile);

    // Write track data (using the track that was generated during initialization).
    FILE *fp;
    // Write left cones.
    fp = fopen("left_cones.csv", "w");
    if (!fp) { perror("Cannot open left_cones.csv"); exit(EXIT_FAILURE); }
    fprintf(fp, "x,y,z\n");
    for (int i = 0; i < controller.nLeftCones; i++) {
        fprintf(fp, "%.6f,%.6f,%.6f\n",
                controller.leftCones[i].x,
                controller.leftCones[i].y,
                controller.leftCones[i].z);
    }
    fclose(fp);

    // Restore terminal settings.
    KeyboardInputHandler_Restore();

    // Write right cones.
    fp = fopen("right_cones.csv", "w");
    if (!fp) { perror("Cannot open right_cones.csv"); exit(EXIT_FAILURE); }
    fprintf(fp, "x,y,z\n");
    for (int i = 0; i < controller.nRightCones; i++) {
        fprintf(fp, "%.6f,%.6f,%.6f\n",
                controller.rightCones[i].x,
                controller.rightCones[i].y,
                controller.rightCones[i].z);
    }
    fclose(fp);

    // Write checkpoints.
    fp = fopen("checkpoints.csv", "w");
    if (!fp) { perror("Cannot open checkpoints.csv"); exit(EXIT_FAILURE); }
    fprintf(fp, "x,y,z\n");
    for (int i = 0; i < controller.nCheckpoints; i++) {
        fprintf(fp, "%.6f,%.6f,%.6f\n",
                controller.checkpointPositions[i].x,
                controller.checkpointPositions[i].y,
                controller.checkpointPositions[i].z);
    }
    fclose(fp);

    printf("Simulation complete.\n");
    printf("Data written to car_telemetry.csv, left_cones.csv, right_cones.csv, and checkpoints.csv\n");

    // Clean up allocated memory.
    if (controller.checkpointPositions != NULL) {
        free(controller.checkpointPositions);
    }
    if (controller.leftCones != NULL) {
        free(controller.leftCones);
    }
    if (controller.rightCones != NULL) {
        free(controller.rightCones);
    }

    return 0;
}