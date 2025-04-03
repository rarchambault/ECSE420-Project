#ifndef LOGIC_H
#define LOGIC_H

#include "Collisions.cuh"  // This header now defines Particle.
#include "Constants.cuh"
#include <cuda_runtime.h>

#ifdef __cplusplus
extern "C" {
#endif

    void runStep(Particle* d_particles[], int numParticles, float deltaTime, float gravity,
        int* d_gridCounters, int* d_gridIndices, Obstacle* d_obstacles,
        int numObstacles, unsigned char* d_framebuffer, int width, int height, int frameNumber, cudaStream_t simStream, cudaStream_t renderStream, cudaEvent_t simEvent,
        int simBuffer, int renderBuffer);

#ifdef __cplusplus
}
#endif

#endif // LOGIC_H