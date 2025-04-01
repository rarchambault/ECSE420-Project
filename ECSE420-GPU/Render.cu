#include "Render.cuh"
#include "Constants.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>


__global__ void renderParticlesKernel(const Particle* particles, int numParticles,
    const Obstacle* obstacles, int numObstacles,
    unsigned char* framebuffer, int width, int height,
    int frameNumber)
{
    int x = blockIdx.x * blockDim.x + threadIdx.x;
    int y = blockIdx.y * blockDim.y + threadIdx.y;
    if (x >= width || y >= height) return;

    int pixelIdx = (y * width + x) * 4;

    // Clear pixel to black.
    framebuffer[pixelIdx + 0] = 0;
    framebuffer[pixelIdx + 1] = 0;
    framebuffer[pixelIdx + 2] = 0;
    framebuffer[pixelIdx + 3] = 255;

    for (int o = 0; o < numObstacles; o++) {
        Obstacle obs = obstacles[o];
        float dx = x - obs.position.x;
        float dy = y - obs.position.y;
        if (dx * dx + dy * dy <= obs.radius * obs.radius) {
            framebuffer[pixelIdx + 0] = 100; // dark gray
            framebuffer[pixelIdx + 1] = 100;
            framebuffer[pixelIdx + 2] = 100;
            framebuffer[pixelIdx + 3] = 255;
            return; // Don't draw particles here
        }
    }

    // Check each particle (naively) to see if it covers this pixel.
    for (int i = 0; i < numParticles; i++) {
        Particle p = particles[i];
        float dx = x - p.position.x;
        float dy = y - p.position.y;
        if ((dx * dx + dy * dy) <= (p.radius * p.radius)) {
            if (frameNumber >= 500) {
                // RED after frame 200
                framebuffer[pixelIdx + 0] = 255;
                framebuffer[pixelIdx + 1] = 0;
                framebuffer[pixelIdx + 2] = 0;
            }
            else {
                // BLUE before frame 200
                framebuffer[pixelIdx + 0] = 0;
                framebuffer[pixelIdx + 1] = 0;
                framebuffer[pixelIdx + 2] = 255;
            }
            break;
        }
    }
}

void renderParticles(const Particle* d_particles, int numParticles,
    const Obstacle* d_obstacles, int numObstacles,
    unsigned char* d_framebuffer, int width, int height, int frameNumber) {
    dim3 threadsPerBlock(16, 16);
    dim3 numBlocks((width + 15) / 16, (height + 15) / 16);
    renderParticlesKernel << <numBlocks, threadsPerBlock >> > (
        d_particles, numParticles,
        d_obstacles, numObstacles,
        d_framebuffer, width, height, frameNumber);
}
