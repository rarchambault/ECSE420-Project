#ifndef SIMULATION_H
#define SIMULATION_H

#include "Collisions.cuh"  // This header now defines Particle.
#include "Constants.cuh"
#include <cuda_runtime.h>

#ifdef __cplusplus
extern "C" {
#endif

    // Run a full simulation step (integration, grid build, collision resolution).
    void runSimulationStep(Particle* d_particles, int numParticles, float deltaTime, float gravity,
        int* d_gridCounters, int* d_gridIndices, Obstacle* obstacles,
        int numObstacles);
#ifdef __cplusplus
}
#endif

#endif // SIMULATION_H