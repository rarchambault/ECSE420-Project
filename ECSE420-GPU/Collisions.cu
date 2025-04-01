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

__global__ void resolveCollisionsKernelSymmetric(Particle* particles,
    int* gridCounters,
    int* gridIndices,
    int numParticles) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numParticles) return;

    // Load particle i (local copy is ok for reading cell info)
    Particle p = particles[i];

    // Determine particle's cell
    int cellX = (int)(p.position.x / GRID_CELL_SIZE);
    int cellY = (int)(p.position.y / GRID_CELL_SIZE);

    // Loop over the 3x3 neighborhood of cells.
    for (int offsetY = -1; offsetY <= 1; offsetY++) {
        for (int offsetX = -1; offsetX <= 1; offsetX++) {
            int neighborX = cellX + offsetX;
            int neighborY = cellY + offsetY;

            // Skip if neighbor cell is outside the grid.
            if (neighborX < 0 || neighborX >= GRID_WIDTH ||
                neighborY < 0 || neighborY >= GRID_HEIGHT)
                continue;

            int neighborCell = neighborY * GRID_WIDTH + neighborX;
            int count = gridCounters[neighborCell];

            // Check each particle in the neighbor cell.
            for (int j = 0; j < count; j++) {
                int otherIndex = gridIndices[neighborCell * MAX_PARTICLES_PER_CELL + j];

                // Ensure each pair is processed only once.
                if (otherIndex <= i)
                    continue;

                // Read the neighbor particle.
                Particle other = particles[otherIndex];

                float dx = other.position.x - p.position.x;
                float dy = other.position.y - p.position.y;
                float distance = sqrtf(dx * dx + dy * dy);
                float minDist = p.radius + other.radius;

                if (distance < minDist && distance > 0.0f) {
                    // Calculate normalized collision normal.
                    float normX = dx / distance;
                    float normY = dy / distance;

                    // Compute symmetric overlap correction.
                    float overlap = (minDist - distance) * 0.5f;
                    atomicAdd(&particles[i].position.x, -overlap * normX);
                    atomicAdd(&particles[i].position.y, -overlap * normY);
                    atomicAdd(&particles[otherIndex].position.x, overlap * normX);
                    atomicAdd(&particles[otherIndex].position.y, overlap * normY);

                    // Compute relative velocity along the collision normal.
                    float relVel = (other.velocity.x - p.velocity.x) * normX +
                        (other.velocity.y - p.velocity.y) * normY;

                    // Only process if particles are moving toward each other.
                    if (relVel < 0.0f) {
                        // Use DAMPING_FACTOR as the restitution coefficient.
                        float restitution = DAMPING_FACTOR;
                        float impulse = -(1.0f + restitution) * relVel / 2.0f; // Equal mass assumed

                        // Atomically update velocities for both particles.
                        atomicAdd(&particles[i].velocity.x, -impulse * normX);
                        atomicAdd(&particles[i].velocity.y, -impulse * normY);
                        atomicAdd(&particles[otherIndex].velocity.x, impulse * normX);
                        atomicAdd(&particles[otherIndex].velocity.y, impulse * normY);
                    }
                }
            }
        }
    }
}



// Host function to launch the collision resolution kernel.
void runCollisionStep(Particle* d_particles, int* d_gridCounters, int* d_gridIndices, int numParticles) {
    int threadsPerBlock = 256;
    int numBlocks = (numParticles + threadsPerBlock - 1) / threadsPerBlock;
    resolveCollisionsKernelSymmetric << <numBlocks, threadsPerBlock >> > (d_particles, d_gridCounters, d_gridIndices, numParticles);
}
