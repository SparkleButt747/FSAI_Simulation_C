#include "centerline.prot.hpp"

#include <SDL.h>

#include "imgui.h"
#include "backends/imgui_impl_sdl2.h"
#include "backends/imgui_impl_sdlrenderer2.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <ctime>
#include <iostream>
#include <limits>
#include <tuple>
#include <utility>
#include <vector>

namespace {

struct Bounds {
    float minX = std::numeric_limits<float>::infinity();
    float maxX = -std::numeric_limits<float>::infinity();
    float minY = std::numeric_limits<float>::infinity();
    float maxY = -std::numeric_limits<float>::infinity();

    void expand(float x, float y)
    {
        if (!std::isfinite(x) || !std::isfinite(y)) {
            return;
        }
        minX = std::min(minX, x);
        maxX = std::max(maxX, x);
        minY = std::min(minY, y);
        maxY = std::max(maxY, y);
    }

    [[nodiscard]] bool valid() const
    {
        return minX <= maxX && minY <= maxY;
    }
};

Bounds computeTrackBounds(
    const TrackResult& track,
    const std::vector<PathNode>& nodes,
    const Point& carFront)
{
    Bounds bounds;

    bounds.expand(static_cast<float>(carFront.x()), static_cast<float>(carFront.y()));

    for (const auto& cone : track.leftCones) {
        bounds.expand(cone.position.x, cone.position.z);
    }
    for (const auto& cone : track.rightCones) {
        bounds.expand(cone.position.x, cone.position.z);
    }
    for (const auto& cone : track.startCones) {
        bounds.expand(cone.position.x, cone.position.z);
    }
    for (const auto& node : nodes) {
        bounds.expand(node.midpoint.x, node.midpoint.y);
    }

    if (!bounds.valid()) {
        bounds.expand(-10.0f, -10.0f);
        bounds.expand(10.0f, 10.0f);
    }

    return bounds;
}

Bounds extendBoundsWithPath(Bounds base, const std::vector<PathNode>& path)
{
    for (const auto& node : path) {
        base.expand(node.midpoint.x, node.midpoint.y);
    }
    return base;
}

SDL_FPoint worldToScreen(
    const Bounds& bounds,
    float worldX,
    float worldY,
    int width,
    int height,
    float margin)
{
    const float spanX = std::max(1e-3f, bounds.maxX - bounds.minX);
    const float spanY = std::max(1e-3f, bounds.maxY - bounds.minY);

    float availableWidth = static_cast<float>(width) - 2.0f * margin;
    float availableHeight = static_cast<float>(height) - 2.0f * margin;
    if (availableWidth <= 0.0f) {
        availableWidth = 1.0f;
    }
    if (availableHeight <= 0.0f) {
        availableHeight = 1.0f;
    }

    const float scale = std::min(availableWidth / spanX, availableHeight / spanY);
    const float offsetX = margin + 0.5f * (availableWidth - scale * spanX);
    const float offsetY = margin + 0.5f * (availableHeight - scale * spanY);

    const float screenX = offsetX + (worldX - bounds.minX) * scale;
    const float screenY = offsetY + (bounds.maxY - worldY) * scale;

    return SDL_FPoint{screenX, screenY};
}

void drawConeSet(
    SDL_Renderer* renderer,
    const std::vector<Transform>& cones,
    SDL_Color color,
    const Bounds& bounds,
    int width,
    int height,
    float margin,
    float markerSize)
{
    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (const auto& cone : cones) {
        const SDL_FPoint p = worldToScreen(bounds, cone.position.x, cone.position.z, width, height, margin);
        const SDL_FRect rect{p.x - markerSize * 0.5f, p.y - markerSize * 0.5f, markerSize, markerSize};
        SDL_RenderFillRectF(renderer, &rect);
    }
}

void drawPath(
    SDL_Renderer* renderer,
    const std::vector<PathNode>& path,
    SDL_Color color,
    const Bounds& bounds,
    int width,
    int height,
    float margin)
{
    if (path.size() < 2) {
        return;
    }

    std::vector<SDL_FPoint> points;
    points.reserve(path.size());
    for (const auto& node : path) {
        points.push_back(worldToScreen(bounds, node.midpoint.x, node.midpoint.y, width, height, margin));
    }

    SDL_SetRenderDrawColor(renderer, color.r, color.g, color.b, color.a);
    for (std::size_t i = 1; i < points.size(); ++i) {
        SDL_RenderDrawLineF(renderer, points[i - 1].x, points[i - 1].y, points[i].x, points[i].y);
    }

    const float markerSize = 4.0f;
    for (const SDL_FPoint& p : points) {
        const SDL_FRect rect{p.x - markerSize * 0.5f, p.y - markerSize * 0.5f, markerSize, markerSize};
        SDL_RenderFillRectF(renderer, &rect);
    }
}

void drawCarFront(
    SDL_Renderer* renderer,
    const Point& carFront,
    const Bounds& bounds,
    int width,
    int height,
    float margin)
{
    const SDL_FPoint p = worldToScreen(bounds, static_cast<float>(carFront.x()), static_cast<float>(carFront.y()), width, height, margin);
    SDL_SetRenderDrawColor(renderer, 220, 70, 70, 255);
    const float halfSize = 6.0f;
    SDL_RenderDrawLineF(renderer, p.x - halfSize, p.y, p.x + halfSize, p.y);
    SDL_RenderDrawLineF(renderer, p.x, p.y - halfSize, p.x, p.y + halfSize);
}

}  // namespace

int main(int argc, char* argv[])
{
    using Clock = std::chrono::high_resolution_clock;

    // Optional seed random number generator.
    if (argc > 1) {
        srand(atoi(argv[1]));
        std::cout << "Seed from cmd args: " << argv[1] << '\n';
    } else {
        unsigned seed = static_cast<unsigned>(time(nullptr));
        std::cout << "Seed: " << seed << '\n';
        srand(seed);
    }

    PathConfig pathConfig = PathConfig(5);
    const int nPoints = pathConfig.resolution;
    PathGenerator pathGen(pathConfig);
    PathResult path = pathGen.generatePath(nPoints);
    TrackGenerator trackGen;
    TrackResult track = trackGen.generateTrack(pathConfig, path);

    Triangulation T;
    Triangulation CarT;

    Point carFront = generateVehicleTriangulation(CarT, track);

    const auto t1 = Clock::now();
    for (const auto& cone : track.leftCones) {
        T.insert(Point(cone.position.x, cone.position.z));
    }
    for (const auto& cone : track.rightCones) {
        T.insert(Point(cone.position.x, cone.position.z));
    }
    const auto t2 = Clock::now();
    const std::chrono::duration<double, std::milli> insertTime = t2 - t1;
    std::cout << "Triangulation insert: " << insertTime.count() << "ms\n";

    Triangulation visibleT;
    getVisibleTrackTriangulation(visibleT, carFront, track);

    CGAL::Graphics_scene scene;
    auto [nodes, adjacency] = generateGraph(visibleT, scene, carFront);

    const Bounds baseBounds = computeTrackBounds(track, nodes, carFront);

    constexpr std::size_t kMaxPathLength = 10;
    int beamWidth = 10;

    auto recomputePath = [&](int beam) {
        const std::size_t width = static_cast<std::size_t>(std::max(1, beam));
        std::vector<PathNode> best = bfsLowestCost(adjacency, nodes, carFront, kMaxPathLength, width);
        float cost = std::numeric_limits<float>::infinity();
        if (best.size() >= 2) {
            cost = calculateCost(best);
        }
        return std::make_pair(std::move(best), cost);
    };

    auto [bestPath, bestCost] = recomputePath(beamWidth);

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        std::cerr << "SDL_Init failed: " << SDL_GetError() << '\n';
        return EXIT_FAILURE;
    }

    SDL_Window* window = SDL_CreateWindow(
        "FSAI Plan",
        SDL_WINDOWPOS_CENTERED,
        SDL_WINDOWPOS_CENTERED,
        1280,
        800,
        SDL_WINDOW_RESIZABLE | SDL_WINDOW_ALLOW_HIGHDPI);
    if (!window) {
        std::cerr << "SDL_CreateWindow failed: " << SDL_GetError() << '\n';
        SDL_Quit();
        return EXIT_FAILURE;
    }

    SDL_Renderer* renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (!renderer) {
        std::cerr << "SDL_CreateRenderer failed: " << SDL_GetError() << '\n';
        SDL_DestroyWindow(window);
        SDL_Quit();
        return EXIT_FAILURE;
    }
    SDL_SetRenderDrawBlendMode(renderer, SDL_BLENDMODE_BLEND);

    IMGUI_CHECKVERSION();
    ImGui::CreateContext();
    ImGuiIO& io = ImGui::GetIO();
    (void)io;
    ImGui::StyleColorsDark();

    ImGui_ImplSDL2_InitForSDLRenderer(window, renderer);
    ImGui_ImplSDLRenderer2_Init(renderer);

    bool running = true;
    constexpr float kViewMargin = 40.0f;

    while (running) {
        SDL_Event event;
        while (SDL_PollEvent(&event)) {
            ImGui_ImplSDL2_ProcessEvent(&event);
            if (event.type == SDL_QUIT) {
                running = false;
            }
            if (event.type == SDL_WINDOWEVENT && event.window.event == SDL_WINDOWEVENT_CLOSE &&
                event.window.windowID == SDL_GetWindowID(window)) {
                running = false;
            }
        }

        ImGui_ImplSDLRenderer2_NewFrame();
        ImGui_ImplSDL2_NewFrame();
        ImGui::NewFrame();

        bool needsRecompute = false;
        CostWeights weights = getCostWeights();

        ImGui::Begin("Path tuning");
        ImGui::Text("Visible nodes: %zu", nodes.size());
        ImGui::Text("Current path nodes: %zu", bestPath.size());
        if (bestPath.size() >= 2 && std::isfinite(bestCost)) {
            ImGui::Text("Path cost: %.3f", bestCost);
        } else {
            ImGui::Text("Path cost: n/a");
        }

        if (ImGui::SliderInt("Beam width", &beamWidth, 1, 50)) {
            needsRecompute = true;
        }

        bool weightsChanged = false;
        weightsChanged |= ImGui::SliderFloat("Angle weight", &weights.angleMax, 0.0f, 1.0f, "%.3f");
        weightsChanged |= ImGui::SliderFloat("Width std weight", &weights.widthStd, 0.0f, 10.0f, "%.2f");
        weightsChanged |= ImGui::SliderFloat("Spacing std weight", &weights.spacingStd, 0.0f, 10.0f, "%.2f");
        weightsChanged |= ImGui::SliderFloat("Color weight", &weights.color, 0.0f, 10.0f, "%.2f");
        weightsChanged |= ImGui::SliderFloat("Range weight", &weights.rangeSq, 0.0f, 5.0f, "%.3f");
        if (ImGui::Button("Reset weights")) {
            weights = defaultCostWeights();
            weightsChanged = true;
        }
        if (weightsChanged) {
            setCostWeights(weights);
            needsRecompute = true;
        }

        ImGui::End();

        if (needsRecompute) {
            std::tie(bestPath, bestCost) = recomputePath(beamWidth);
        }

        int renderWidth = 0;
        int renderHeight = 0;
        SDL_GetRendererOutputSize(renderer, &renderWidth, &renderHeight);

        SDL_SetRenderDrawColor(renderer, 18, 22, 30, 255);
        SDL_RenderClear(renderer);

        const Bounds bounds = extendBoundsWithPath(baseBounds, bestPath);

        drawConeSet(renderer, track.leftCones, SDL_Color{30, 144, 255, 255}, bounds, renderWidth, renderHeight, kViewMargin, 6.0f);
        drawConeSet(renderer, track.rightCones, SDL_Color{255, 215, 0, 255}, bounds, renderWidth, renderHeight, kViewMargin, 6.0f);
        drawConeSet(renderer, track.startCones, SDL_Color{255, 140, 0, 255}, bounds, renderWidth, renderHeight, kViewMargin, 8.0f);

        drawPath(renderer, bestPath, SDL_Color{255, 255, 255, 255}, bounds, renderWidth, renderHeight, kViewMargin);
        drawCarFront(renderer, carFront, bounds, renderWidth, renderHeight, kViewMargin);

        ImGui::Render();
        ImGui_ImplSDLRenderer2_RenderDrawData(ImGui::GetDrawData(), renderer);
        SDL_RenderPresent(renderer);
    }

    ImGui_ImplSDLRenderer2_Shutdown();
    ImGui_ImplSDL2_Shutdown();
    ImGui::DestroyContext();

    SDL_DestroyRenderer(renderer);
    SDL_DestroyWindow(window);
    SDL_Quit();

    return EXIT_SUCCESS;
}
