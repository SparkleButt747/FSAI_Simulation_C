#ifndef GRAPHICS_H
#define GRAPHICS_H

#ifdef __cplusplus
extern "C" {
#endif

#include <SDL.h>

typedef struct {
    SDL_Window* window;
    SDL_Renderer* renderer;
    int width;
    int height;
} Graphics;

// Initializes SDL, creates a window and renderer. Returns 0 on success.
int Graphics_Init(Graphics* g, const char* title, int width, int height);

// Clears the screen (white background).
void Graphics_Clear(Graphics* g);

// Draws grid lines with a specified spacing (in pixels).
void Graphics_DrawGrid(Graphics* g, int gridSize);

// Draws the car as a filled circle with a heading indicator.
// (x,y) are screen coordinates; radius in pixels; yaw in radians.
void Graphics_DrawCar(Graphics* g, float x, float y, float radius, float yaw);

// Draws a filled circle at (centreX, centreY) with given radius.
void Graphics_DrawFilledCircle(Graphics* g, int centreX, int centreY, int radius);

// Draws a line segment between (x1, y1) and (x2, y2).
void Graphics_DrawSegment(Graphics* g, float x1, float y1,  float x2, float y2,
                          float render_scale, int red, int green, int blue);
// Presents the current frame.
void Graphics_Present(Graphics* g);

// Handles window-specific SDL events to keep renderer state synchronized.
void Graphics_HandleWindowEvent(Graphics* g, const SDL_Event* event);

// Cleans up SDL resources.
void Graphics_Cleanup(Graphics* g);

#ifdef __cplusplus
}
#endif

#endif // GRAPHICS_H
