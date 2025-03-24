#include "raylib.h"
#include "simulation.h"
#include "renderer.h"
#include "constants.h"

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Particle Simulator");

    InitSimulation();
    InitRenderer();

    while (!WindowShouldClose()) {
        UpdateSimulation();

        Render();
    }

    CleanupSimulation();
    CleanupRenderer();
    CloseWindow();
    return 0;
}