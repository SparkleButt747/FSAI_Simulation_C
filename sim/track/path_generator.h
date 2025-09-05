#ifndef PATH_GENERATOR_H
#define PATH_GENERATOR_H

#include "path_config.h"
#ifdef __cplusplus
#include <complex>
typedef std::complex<double> cdouble;
#else
#include <complex.h>
typedef double complex cdouble;
#endif

/* Structure holding the generated path data.
   - points: scaled track positions (represented as complex numbers; the real part is x, the imaginary part is z).
   - normals: corresponding normals for each point.
   - cornerRadii: the (scaled) corner radii at each point.
   The caller is responsible for freeing the allocated arrays.
*/
typedef struct {
    int nPoints;
    cdouble *points;
    cdouble *normals;
    double *cornerRadii;
} PathResult;

#ifdef __cplusplus
extern "C" {
#endif

/* Generates the track path based on the given configuration and number of points.
   The function allocates memory for the output arrays. */
PathResult generate_path_with_params(const PathConfig *config, int nPoints);

#ifdef __cplusplus
}
#endif

#endif // PATH_GENERATOR_H
