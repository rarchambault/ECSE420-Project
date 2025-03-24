#include "raylib.h"
#include "simulation.h"
#include "renderer.h"

#define WINDOW_WIDTH 1280
#define WINDOW_HEIGHT 800

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Particle Simulator");

    InitSimulation();
    InitRenderer();

    while (!WindowShouldClose()) {
        UpdateSimulation();

        BeginDrawing();
        ClearBackground(BLACK);
        RenderParticles();
        EndDrawing();
    }

    CleanupSimulation();
    CleanupRenderer();
    CloseWindow();
    return 0;
}