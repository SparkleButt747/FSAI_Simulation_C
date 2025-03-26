#include "path_generator.h"
#include <stdlib.h>
#include <math.h>
#include <complex.h>
#include <stdio.h>

/* ============================================================================
   Static Helper Functions
   ============================================================================ */

/* Computes corner radii for each point using finite differences.
   dt: time step (or angular step) between successive points.
   dPdt: an array of derivatives of the path (as complex numbers).
   nPoints: the number of points.
   Returns a dynamically allocated array of doubles (size nPoints) which the caller must free.
*/
static double* compute_corner_radii(double dt, const double complex *dPdt, int nPoints) {
    double complex *ddPdt = malloc(nPoints * sizeof(double complex));
    if (!ddPdt) {
        perror("malloc ddPdt");
        exit(EXIT_FAILURE);
    }
    
    for (int i = 0; i < nPoints - 1; i++) {
        ddPdt[i] = (dPdt[i + 1] - dPdt[i]) / dt;
    }
    ddPdt[nPoints - 1] = (dPdt[0] - dPdt[nPoints - 1]) / dt;
    
    double *cornerRadii = malloc(nPoints * sizeof(double));
    if (!cornerRadii) {
        perror("malloc cornerRadii");
        exit(EXIT_FAILURE);
    }
    
    for (int i = 0; i < nPoints; i++) {
        double num = pow(cabs(dPdt[i]), 3);
        double denom = cimag(conj(dPdt[i]) * ddPdt[i]);
        /* Guard against division by zero */
        if (denom == 0) {
            cornerRadii[i] = 0;
        } else {
            cornerRadii[i] = num / denom;
        }
    }
    
    free(ddPdt);
    return cornerRadii;
}

/* ============================================================================
   Main Function: generate_path_with_params
   ============================================================================ */

PathResult generate_path_with_params(const PathConfig *config, int nPoints) {
    double minCornerRadius = config->minCornerRadius;
    int maxFrequency = config->maxFrequency;
    double amplitude = config->amplitude;
    
    /* Allocate arrays for intermediate computations */
    double complex *z      = malloc(nPoints * sizeof(double complex));
    double complex *waves  = malloc(nPoints * sizeof(double complex));
    double complex *dwaves = malloc(nPoints * sizeof(double complex));
    double complex *zPow   = malloc(nPoints * sizeof(double complex));
    
    if (!z || !waves || !dwaves || !zPow) {
        perror("malloc intermediate arrays");
        exit(EXIT_FAILURE);
    }
    
    /* Initialize waves and dwaves to zero */
    for (int i = 0; i < nPoints; i++) {
        waves[i]  = 0.0 + 0.0 * I;
        dwaves[i] = 0.0 + 0.0 * I;
    }
    
    /* Sample points on the unit circle */
    for (int t = 0; t < nPoints; t++) {
        double angle = 2 * M_PI * t / nPoints;
        z[t] = cexp(I * angle);
    }
    
    /* Generate wave perturbations for frequencies 2 to maxFrequency */
    for (int frequency = 2; frequency <= maxFrequency; frequency++) {
        /* Random phase in [0, 2pi) */
        double random_phase = ((double)rand() / RAND_MAX) * 2 * M_PI;
        double complex phase = cexp(I * random_phase);
        
        /* Compute zPow[t] = (z[t])^frequency */
        for (int t = 0; t < nPoints; t++) {
            zPow[t] = cpow(z[t], frequency);
        }
        
        for (int t = 0; t < nPoints; t++) {
            /* 
             waves[t] += z[t] * ( zPow[t] / (phase * (frequency + 1)) + phase / (zPow[t] * (frequency - 1)) )
             dwaves[t] += ( zPow[t] / phase ) - ( phase / zPow[t] )
            */
            waves[t]  += z[t] * ( zPow[t] / (phase * (frequency + 1)) + phase / (zPow[t] * (frequency - 1)) );
            dwaves[t] += ( zPow[t] / phase ) - ( phase / zPow[t] );
        }
    }
    
    /* Scale the wave perturbation by amplitude */
    double complex *scaledwaves = malloc(nPoints * sizeof(double complex));
    if (!scaledwaves) {
        perror("malloc scaledwaves");
        exit(EXIT_FAILURE);
    }
    for (int t = 0; t < nPoints; t++) {
        scaledwaves[t] = waves[t] * amplitude;
    }
    
    /* Compute the final path points: points = z + scaledwaves */
    double complex *points = malloc(nPoints * sizeof(double complex));
    if (!points) {
        perror("malloc points");
        exit(EXIT_FAILURE);
    }
    for (int t = 0; t < nPoints; t++) {
        points[t] = z[t] + scaledwaves[t];
    }
    
    /* Compute derivative dPdt for use in normal and corner radius calculation */
    double complex *dPdt = malloc(nPoints * sizeof(double complex));
    if (!dPdt) {
        perror("malloc dPdt");
        exit(EXIT_FAILURE);
    }
    for (int t = 0; t < nPoints; t++) {
        dPdt[t] = I * z[t] * (1 + amplitude * dwaves[t]);
    }
    
    /* Compute normals: normals[t] = (I * dPdt[t]) / |dPdt[t]| */
    double complex *normals = malloc(nPoints * sizeof(double complex));
    if (!normals) {
        perror("malloc normals");
        exit(EXIT_FAILURE);
    }
    for (int t = 0; t < nPoints; t++) {
        double mag = cabs(dPdt[t]);
        if (mag == 0) mag = 1;  /* Avoid division by zero */
        normals[t] = (I * dPdt[t]) / mag;
    }
    
    /* Compute corner radii using a helper function */
    double dt = 2 * M_PI / nPoints;
    double *cornerRadii = compute_corner_radii(dt, dPdt, nPoints);
    
    /* Find the minimum absolute corner radius */
    double minValue = 1e308;  /* Use a very large number initially */
    for (int t = 0; t < nPoints; t++) {
        double absVal = fabs(cornerRadii[t]);
        if (absVal < minValue) {
            minValue = absVal;
        }
    }
    
    double scale = minCornerRadius / minValue;
    
    /* Scale points and corner radii */
    double complex *scaledPoints = malloc(nPoints * sizeof(double complex));
    double *scaledCornerRadii = malloc(nPoints * sizeof(double));
    if (!scaledPoints || !scaledCornerRadii) {
        perror("malloc scaledPoints/scaledCornerRadii");
        exit(EXIT_FAILURE);
    }
    for (int t = 0; t < nPoints; t++) {
        scaledPoints[t] = points[t] * scale;
        scaledCornerRadii[t] = cornerRadii[t] * scale;
    }
    
    /* Free intermediate arrays */
    free(z);
    free(waves);
    free(dwaves);
    free(zPow);
    free(scaledwaves);
    free(dPdt);
    free(cornerRadii);
    free(points);
    
    /* Package the results */
    PathResult result;
    result.nPoints = nPoints;
    result.points = scaledPoints;
    result.normals = normals;
    result.cornerRadii = scaledCornerRadii;
    
    return result;
}
