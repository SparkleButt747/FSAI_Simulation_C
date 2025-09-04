#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "path_config.h"
#include <complex.h>

/* Structure holding the generated path data.
   - points: scaled track positions (represented as complex numbers; the real part is x, the imaginary part is z).
   - normals: corresponding normals for each point.
   - cornerRadii: the (scaled) corner radii at each point.
   The caller is responsible for freeing the allocated arrays.
*/
typedef struct {
    int nPoints;
    double complex *points;
    double complex *normals;
    double *cornerRadii;
} PathResult;

/* Generates the track path based on the given configuration and number of points.
   The function allocates memory for the output arrays. */
PathResult generate_path_with_params(const PathConfig *config, int nPoints);

#endif // PATH_GENERATOR_H
