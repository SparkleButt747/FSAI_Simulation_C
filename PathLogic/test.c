#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include "path_config.h"
#include "path_generator.h"
#include "track_generator.h"

int main(void) {
    /* Seed the random number generator */
    srand((unsigned) time(NULL));

    /* Initialize configuration with default values */
    PathConfig config;
    init_default_path_config(&config);

    /* Determine the number of points to sample; using the computed resolution */
    int nPoints = config.resolution;

    /* Generate the path data (points, normals, corner radii) */
    PathResult path = generate_path_with_params(&config, nPoints);

    /* Generate the track: left cones, right cones, and checkpoints */
    TrackResult track = generate_track(&config, &path);

    /* Write the cone and checkpoint data to CSV files */
    FILE *fp;

    /* Write left cones to CSV */
    fp = fopen("left_cones.csv", "w");
    if (fp == NULL) { perror("Cannot open left_cones.csv"); exit(EXIT_FAILURE); }
    fprintf(fp, "x,y,z,rotation\n");
    for (int i = 0; i < track.nLeftCones; i++) {
        fprintf(fp, "%lf,%lf,%lf,%lf\n",
                track.leftCones[i].position.x,
                track.leftCones[i].position.y,
                track.leftCones[i].position.z,
                track.leftCones[i].yaw);
    }
    fclose(fp);

    /* Write right cones to CSV */
    fp = fopen("right_cones.csv", "w");
    if (fp == NULL) { perror("Cannot open right_cones.csv"); exit(EXIT_FAILURE); }
    fprintf(fp, "x,y,z,rotation\n");
    for (int i = 0; i < track.nRightCones; i++) {
        fprintf(fp, "%lf,%lf,%lf,%lf\n",
                track.rightCones[i].position.x,
                track.rightCones[i].position.y,
                track.rightCones[i].position.z,
                track.rightCones[i].yaw);
    }
    fclose(fp);

    /* Write checkpoints to CSV */
    fp = fopen("checkpoints.csv", "w");
    if (fp == NULL) { perror("Cannot open checkpoints.csv"); exit(EXIT_FAILURE); }
    fprintf(fp, "x,y,z,rotation\n");
    for (int i = 0; i < track.nCheckpoints; i++) {
        fprintf(fp, "%lf,%lf,%lf,%lf\n",
                track.checkpoints[i].position.x,
                track.checkpoints[i].position.y,
                track.checkpoints[i].position.z,
                track.checkpoints[i].yaw);
    }
    fclose(fp);

    printf("Data written to left_cones.csv, right_cones.csv, and checkpoints.csv\n");

    /* Optionally, call a Python script to plot the data.
       This assumes a script named 'plot_track.py' exists in the same directory.
       If the script is not available or the system call fails, a message is printed.
    */
    int ret = system("python plot_track.py");
    if (ret != 0) {
        fprintf(stderr, "Graphing tool did not run successfully. Please run plot_track.py manually.\n");
    }

    /* Free dynamically allocated memory */
    free(path.points);
    free(path.normals);
    free(path.cornerRadii);
    free(track.leftCones);
    free(track.rightCones);
    free(track.checkpoints);

    return 0;
}
