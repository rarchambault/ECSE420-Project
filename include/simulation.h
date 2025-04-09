#ifndef SIMULATION_H
#define SIMULATION_H

#include "collision.h"
#include <pthread.h>

#define NB_PARTICLES 1000
#define NB_OBSTACLES 6

#define NUM_THREADS_CPU 8
#define GRID_WIDTH 4 // Number of grid cells across the window's width
#define GRID_HEIGHT 4 // Number of grid cells across the window's height
#define GRID_CELL_WIDTH (WINDOW_WIDTH / GRID_WIDTH) // Size of each grid cell in pixels
#define GRID_CELL_HEIGHT (WINDOW_HEIGHT / GRID_HEIGHT) // Size of each grid cell in pixels

#define MAX_PARTICLES_PER_CELL 256 // For openCl

typedef enum {
    EXECUTION_SEQUENTIAL,
    EXECUTION_CPU_THREADING,
    EXECUTION_OPENCL_GPU,
    EXECUTION_CUDA_GPU
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

void InitSimulation();
void UpdateSimulation();
void CleanupSimulation();
Particle* GetParticles();
Obstacle* GetObstacles();

#endif