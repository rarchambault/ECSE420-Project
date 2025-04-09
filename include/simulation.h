#ifndef SIMULATION_H
#define SIMULATION_H

#include "collision.h"
#include <pthread.h>

#define NB_PARTICLES 6000
#define NB_OBSTACLES 6

#define NUM_THREADS_CPU 8
#define GRID_WIDTH 80 // Number of grid cells across the window's width
#define GRID_HEIGHT 60 // Number of grid cells across the window's height
#define GRID_CELL_WIDTH (WINDOW_WIDTH / GRID_WIDTH) // Size of each grid cell in pixels
#define GRID_CELL_HEIGHT (WINDOW_HEIGHT / GRID_HEIGHT) // Size of each grid cell in pixels

#define MAX_PARTICLES_PER_CELL 256

typedef enum {
    EXECUTION_SEQUENTIAL,
    EXECUTION_CPU_THREADING,
    EXECUTION_GPU_OPENCL,
    EXECUTION_CPU_GRAPHICS_OPENCL,
    EXECUTION_CPU_OPENCL,
    EXECUTION_GPU_CUDA
} ExecutionMode;

extern ExecutionMode executionMode;

typedef struct {
    int count;
    Particle* particles[NB_PARTICLES]; // Stores pointers to particles in this cell
} GridCell;

typedef struct {
    int count;
    int indices[MAX_PARTICLES_PER_CELL];
} GridCellGPU;

typedef struct {
    int threadIndex;
} CpuThreadData;

void InitSimulation();
void UpdateSimulation();
void CleanupSimulation();
Particle* GetParticles();
Obstacle* GetObstacles();

#endif