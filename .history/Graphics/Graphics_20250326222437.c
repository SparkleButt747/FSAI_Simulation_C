#include "Graphics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

// Initializes SDL, creates a window and renderer.
int Graphics_Init(Graphics* g, const char* title, int width, int height) {
    printf("Initializing SDL...\n");
    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "SDL_Init Error: %s\n", SDL_GetError());
        return -1;
    }
    g->width = width;
    g->height = height;
    g->window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height, SDL_WINDOW_SHOWN);
    if (!g->window) {
        fprintf(stderr, "SDL_CreateWindow Error: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }
    g->renderer = SDL_CreateRenderer(g->window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!g->renderer) {
        SDL_DestroyWindow(g->window);
        fprintf(stderr, "SDL_CreateRenderer Error: %s\n", SDL_GetError());
        SDL_Quit();
        return -1;
    }
    return 0;
}

// Clears the screen with a white background.
void Graphics_Clear(Graphics* g) {
    printf("Clearing screen...\n");
    SDL_SetRenderDrawColor(g->renderer, 255, 255, 255, 255);
    SDL_RenderClear(g->renderer);
}

// Draws grid lines on the screen using the given spacing.
void Graphics_DrawGrid(Graphics* g, int gridSize) {
    printf("Drawing grid with spacing %d...\n", gridSize);
    SDL_SetRenderDrawColor(g->renderer, 200, 200, 200, 255); // light gray color
    for (int x = 0; x < g->width; x += gridSize) {
        SDL_RenderDrawLine(g->renderer, x, 0, x, g->height);
    }
    for (int y = 0; y < g->height; y += gridSize) {
        SDL_RenderDrawLine(g->renderer, 0, y, g->width, y);
    }
}

// Helper function to draw a filled circle using horizontal lines.
 void drawFilledCircle(SDL_Renderer* renderer, int centreX, int centreY, int radius) {
    printf("Drawing filled circle at (%d,%d) with radius %d...\n", centreX, centreY, radius);
    for (int y = -radius; y <= radius; y++) {
        int dx = (int) sqrt(radius * radius - y * y);
        SDL_RenderDrawLine(renderer, centreX - dx, centreY + y, centreX + dx, centreY + y);
    }
}

// Draws a filled circle representing the car with a heading line.
void Graphics_DrawCar(Graphics* g, float x, float y, float radius, float yaw) {
    printf("Drawing car at (%.2f,%.2f) with radius %.2f and yaw %.2f...\n", x, y, radius, yaw);
    // Draw the car (blue filled circle).
    SDL_SetRenderDrawColor(g->renderer, 0, 0, 255, 255);
    drawFilledCircle(g->renderer, (int)x, (int)y, (int)radius);
    
    // Draw heading: a red line from the center extending in the yaw direction.
    float lineLength = radius;
    int x2 = (int)(x + lineLength * cos(yaw));
    int y2 = (int)(y + lineLength * sin(yaw));
    SDL_SetRenderDrawColor(g->renderer, 255, 0, 0, 255);
    SDL_RenderDrawLine(g->renderer, (int)x, (int)y, x2, y2);
}


// Presents the rendered frame.
void Graphics_Present(Graphics* g) {
    printf("Presenting frame...\n");
    SDL_RenderPresent(g->renderer);
}

// Cleans up SDL resources.
void Graphics_Cleanup(Graphics* g) {
    printf("Cleaning up SDL resources...\n");
    if (g->renderer)
        SDL_DestroyRenderer(g->renderer);
    if (g->window)
        SDL_DestroyWindow(g->window);
    SDL_Quit();
}
