#ifndef RENDER_H
#define RENDER_H

#include "Collisions.cuh"  // Uses Particle defined in collisions.h.
#include "Constants.cuh"
#include <cuda_runtime.h>

#ifdef __cplusplus
extern "C" {
#endif

    // Render function: writes particle colors to a framebuffer.
    void renderParticles(const Particle* d_particles, int numParticles,
        const Obstacle* d_obstacles, int numObstacles,
        unsigned char* d_framebuffer, int width, int height, int frameNumber);

#ifdef __cplusplus
}
#endif

#endif // RENDER_H