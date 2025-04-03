#ifndef SIMULATION_H
#define SIMULATION_H

#include "collision.h"
#include <pthread.h>

#define NB_PARTICLES 500
#define NB_OBSTACLES 6
#define NUM_THREADS_CPU 8

typedef struct {
    int start;
    int end;
} CpuThreadData;

typedef enum {
    EXECUTION_SEQUENTIAL,
    EXECUTION_CPU_THREADING
} ExecutionMode;

extern ExecutionMode executionMode;

void InitSimulation();
void UpdateSimulation();
void CleanupSimulation();
Particle* GetParticles();
Obstacle* GetObstacles();

#endif