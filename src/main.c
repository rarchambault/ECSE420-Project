#include "raylib.h"
#include "simulation.h"
#include "renderer.h"
#include "constants.h"
#include <time.h>
#include <stdio.h>

ExecutionMode executionMode = EXECUTION_CPU_GRAPHICS_OPENCL;

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Particle Simulator");

    InitSimulation();
    InitRenderer();

    int frame = 0;
    int maxFrame = 200;

    double startTime = GetTime();

    while (!WindowShouldClose()) {
        if (frame >= maxFrame) {
			break;
		}

        UpdateSimulation();

        Render();

        frame++;
    }

    double endTime = GetTime();
    double elapsedTime = endTime - startTime;
    printf("--- TIME: %.4f sec ---\n", elapsedTime);

    CleanupSimulation();
    CleanupRenderer();
    CloseWindow();
    return 0;
}