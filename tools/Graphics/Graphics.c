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
    g->window = SDL_CreateWindow(title, SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED, width, height,
                                 SDL_WINDOW_SHOWN | SDL_WINDOW_RESIZABLE);
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

void Graphics_HandleWindowEvent(Graphics* g, const SDL_Event* event) {
    if (!g || !event) {
        return;
    }
    if (event->type != SDL_WINDOWEVENT) {
        return;
    }
    if (event->window.windowID != SDL_GetWindowID(g->window)) {
        return;
    }
    switch (event->window.event) {
        case SDL_WINDOWEVENT_SIZE_CHANGED:
        case SDL_WINDOWEVENT_RESIZED: {
            int output_w = 0;
            int output_h = 0;
            if (SDL_GetRendererOutputSize(g->renderer, &output_w, &output_h) == 0) {
                g->width = output_w;
                g->height = output_h;
            } else {
                g->width = event->window.data1;
                g->height = event->window.data2;
            }
            SDL_RenderSetViewport(g->renderer, NULL);
            break;
        }
        default:
            break;
    }
}

// Clears the screen with a black background to keep overlays readable.
void Graphics_Clear(Graphics* g) {
    SDL_SetRenderDrawColor(g->renderer, 0, 0, 0, 255);
    SDL_RenderClear(g->renderer);
}

// Draws grid lines on the screen using the given spacing.
void Graphics_DrawGrid(Graphics* g, int gridSize) {
    SDL_SetRenderDrawColor(g->renderer, 200, 200, 200, 255); // light gray color
    for (int x = 0; x < g->width; x += gridSize) {
        SDL_RenderDrawLine(g->renderer, x, 0, x, g->height);
    }
    for (int y = 0; y < g->height; y += gridSize) {
        SDL_RenderDrawLine(g->renderer, 0, y, g->width, y);
    }
}

// Helper function to draw a filled circle using horizontal lines.
static void drawFilledCircle(SDL_Renderer* renderer, int centreX, int centreY, int radius) {
    //printf("Drawing filled circle at (%d,%d) with radius %d...\n", centreX, centreY, radius);
    for (int y = -radius; y <= radius; y++) {
        int dx = (int) sqrt(radius * radius - y * y);
        SDL_RenderDrawLine(renderer, centreX - dx, centreY + y, centreX + dx, centreY + y);
    }
}

void Graphics_DrawFilledCircle(Graphics* g, int centreX, int centreY, int radius) {
    //printf("Drawing filled circle at (%d,%d) with radius %d...\n", centreX, centreY, radius);
    for (int y = -radius; y <= radius; y++) {
        int dx = (int) sqrt(radius * radius - y * y);
        SDL_RenderDrawLine(g->renderer, centreX - dx, centreY + y, centreX + dx, centreY + y);
    }
}

// Draws the car as a small yellow triangle aligned with heading.
void Graphics_DrawCar(Graphics* g, float x, float y, float radius, float yaw) {
    const float tip_x = x + radius * cosf(yaw);
    const float tip_y = y + radius * sinf(yaw);

    const float base_angle = 2.0f * 3.14159265358979323846f / 3.0f;
    const float left_x = x + radius * cosf(yaw + base_angle);
    const float left_y = y + radius * sinf(yaw + base_angle);
    const float right_x = x + radius * cosf(yaw - base_angle);
    const float right_y = y + radius * sinf(yaw - base_angle);

    SDL_SetRenderDrawColor(g->renderer, 255, 215, 0, 255);
    SDL_RenderDrawLineF(g->renderer, tip_x, tip_y, left_x, left_y);
    SDL_RenderDrawLineF(g->renderer, left_x, left_y, right_x, right_y);
    SDL_RenderDrawLineF(g->renderer, right_x, right_y, tip_x, tip_y);
}

void Graphics_DrawSegment(Graphics* g, float x1, float y1,  float x2, float y2, int red, int green, int blue) {
    x1 = x1 * K_RENDER_SCALE + g->width / 2.0f;
    y1 = y1 * K_RENDER_SCALE + g->height / 2.0f;
    x2 = x2 * K_RENDER_SCALE + g->width / 2.0f;
    y2 = y2 * K_RENDER_SCALE + g->height / 2.0f;
    SDL_SetRenderDrawColor(g->renderer, red, green, blue, 255);
    SDL_RenderDrawLine(g->renderer, x1, y1, x2, y2);
}

// Presents the rendered frame.
void Graphics_Present(Graphics* g) {
    //printf("Presenting frame...\n");
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
