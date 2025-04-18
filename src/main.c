#include "raylib.h"
#include "simulation.h"
#include "renderer.h"
#include "constants.h"
#include <time.h>
#include <stdio.h>

ExecutionMode executionMode = EXECUTION_GPU_OPENCL;

int main() {
    SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Particle Simulator");

    InitSimulation();
    InitRenderer();

    int frame = 0;
    int maxFrame = 200;

    clock_t startTime = clock();

    while (frame < maxFrame) {

        UpdateSimulation();

        Render();

        frame++;
    }

    clock_t endTime = clock();
    double elapsedTime = (double)(endTime - startTime) / CLOCKS_PER_SEC;
    printf("--- TIME: %.4f sec ---\n", elapsedTime);
    
    CleanupSimulation();
    CleanupRenderer();
    CloseWindow();
    return 0;
}