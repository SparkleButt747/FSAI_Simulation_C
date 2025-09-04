#ifndef TRACK_GENERATOR_H
#define TRACK_GENERATOR_H

#include "path_config.h"
#include "path_generator.h"
#include <complex.h>

#include "../common/physics/Transform.h"      // from PhysicsEngine
#include "../common/physics/Vector.h"      // from PhysicsEngine


#ifdef __cplusplus
extern "C" {
#endif



/* 
 * TrackResult holds the generated cone and checkpoint transforms.
 * The caller is responsible for freeing each dynamically allocated array.
 */
typedef struct {
    int nLeftCones;
    Transform* leftCones;
    int nRightCones;
    Transform* rightCones;
    int nCheckpoints;
    Transform* checkpoints;
} TrackResult;

/* 
 * Generates the track data (left cones, right cones, and checkpoints) based on the provided
 * configuration and path result. This function uses a spacing algorithm similar in spirit to the
 * original C# PlaceCones routine.
 */
TrackResult generate_track(const PathConfig *config, const PathResult *path);

#ifdef __cplusplus
}
#endif

#endif // TRACK_GENERATOR_H
