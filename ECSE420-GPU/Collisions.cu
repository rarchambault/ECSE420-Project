#include "Collisions.cuh"
#include "Constants.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

// Collision resolution kernel: For each particle, check its neighborhood in the grid
// and resolve any overlaps with other particles.
__global__ void resolveCollisionsKernel(Particle* particles, int* gridCounters, int* gridIndices, int numParticles) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numParticles) return;

    Particle p = particles[i];

    // Determine particle's cell
    int cellX = (int)(p.position.x / GRID_CELL_SIZE);
    int cellY = (int)(p.position.y / GRID_CELL_SIZE);

    // Loop over 3x3 neighborhood
    for (int offsetY = -1; offsetY <= 1; offsetY++) {
        for (int offsetX = -1; offsetX <= 1; offsetX++) {
            int neighborX = cellX + offsetX;
            int neighborY = cellY + offsetY;

            // Skip if outside grid
            if (neighborX < 0 || neighborX >= GRID_WIDTH ||
                neighborY < 0 || neighborY >= GRID_HEIGHT)
                continue;

            int neighborIndex = neighborY * GRID_WIDTH + neighborX;
            int count = gridCounters[neighborIndex];

            for (int j = 0; j < count; j++) {
                int otherIndex = gridIndices[neighborIndex * MAX_PARTICLES_PER_CELL + j];
                if (otherIndex == i) continue;

                Particle other = particles[otherIndex];

                float dx = other.position.x - p.position.x;
                float dy = other.position.y - p.position.y;
                float distance = sqrtf(dx * dx + dy * dy);
                float minDist = p.radius + other.radius;

                if (distance < minDist && distance > 0.0f) {
                    // Normalize collision normal
                    float normX = dx / distance;
                    float normY = dy / distance;

                    // Push particles apart
                    float overlap = (minDist - distance) * 0.5f;
                    p.position.x -= overlap * normX;
                    p.position.y -= overlap * normY;

                    // Reflect velocity
                    float vDotN = p.velocity.x * normX + p.velocity.y * normY;
                    if (vDotN > 0.0f) vDotN = 0.0f;

                    p.velocity.x -= 2.0f * vDotN * normX;
                    p.velocity.y -= 2.0f * vDotN * normY;

                    // Dampen
                    p.velocity.x *= DAMPING_FACTOR;
                    p.velocity.y *= DAMPING_FACTOR;

                    // Wake-up nudge if velocity is too low
                    float speed2 = p.velocity.x * p.velocity.x + p.velocity.y * p.velocity.y;
                    if (speed2 < 1e-4f) {

                        p.velocity.x += 0.1f * normX;
                        p.velocity.y += 0.1f * normY;
                    }
                }
            }
        }
    }

    float jitter = 0.05f;
    p.velocity.x += jitter * ((i % 2 == 0) ? 1.0f : -1.0f); // pseudo-random X
    p.velocity.y += jitter * ((i % 3 == 0) ? 1.0f : -1.0f); // pseudo-random Y

    particles[i] = p;
}

