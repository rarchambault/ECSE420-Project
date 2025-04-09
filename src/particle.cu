#include "particle.h"
#include "simulation.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

__device__ float vector2_distance_device(Vector2 a, Vector2 b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

__device__ int check_collision_circles_device(Vector2 center1, float radius1, Vector2 center2, float radius2) {
    return vector2_distance_device(center1, center2) <= (radius1 + radius2);
}

__global__ void updateParticlesKernel(Particle* particles, Obstacle* obstacles) {
    // Calculate global thread index
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    // If the thread index is out of bounds, return
    if (idx >= NB_PARTICLES) return;

    Particle* particle = &particles[idx];

    // Update particle velocity with gravity and position
    particle->velocity.y += GRAVITY;
    particle->position.x += particle->velocity.x;
    particle->position.y += particle->velocity.y;

    // Resolve collisions with boundaries
    if (particle->position.x <= MAX_LEFT) {
        particle->position.x = MAX_LEFT;
        particle->velocity.x *= -DAMPING_FACTOR;
    }
    if (particle->position.x >= MAX_RIGHT) {
        particle->position.x = MAX_RIGHT;
        particle->velocity.x *= -DAMPING_FACTOR;
    }
    if (particle->position.y <= MAX_TOP) {
        particle->position.y = MAX_TOP;
        particle->velocity.y *= -DAMPING_FACTOR;
    }
    if (particle->position.y >= MAX_BOTTOM) {
        particle->position.y = MAX_BOTTOM;
        particle->velocity.y *= -DAMPING_FACTOR;
    }

    // Resolve collisions with obstacles
    for (int j = 0; j < NB_OBSTACLES; j++) {
        Obstacle* obstacle = &obstacles[j];

        if (check_collision_circles_device(particle->position, particle->radius, obstacle->position, obstacle->radius)) {

            float dx = particle->position.x - obstacle->position.x;
            float dy = particle->position.y - obstacle->position.y;
            float distance = sqrtf(dx * dx + dy * dy);

            // Resolve overlap by pushing the particle away from the obstacle
            float overlap = (particle->radius + obstacle->radius) - distance;
            float normX = dx / distance;
            float normY = dy / distance;

            // Push the particle outside of the obstacle
            particle->position.x += overlap * normX;
            particle->position.y += overlap * normY;

            // Reflect the particle's velocity based on the normal direction
            float dotProduct = (particle->velocity.x * normX) + (particle->velocity.y * normY);

            // Reflect velocity based on the normal direction
            particle->velocity.x -= 2 * dotProduct * normX;
            particle->velocity.y -= 2 * dotProduct * normY;

            // Apply the damping factor after collision to reduce velocity
            particle->velocity.x *= DAMPING_FACTOR;
            particle->velocity.y *= DAMPING_FACTOR;
        }
    }
}

__device__ void ResolveCollision(Particle* particles, int i, int j) {
    // Distance and direction between particles
    float dx = particles[j].position.x - particles[i].position.x;
    float dy = particles[j].position.y - particles[i].position.y;
    float distance = sqrt(dx * dx + dy * dy);
    float minDist = particles[i].radius + particles[j].radius;

    if (distance < minDist) {

        // Handle perfect overlap
        if (distance < 1e-6f) {
            dx = 0.01f;
            dy = 0.01f;
            distance = sqrt(dx * dx + dy * dy);
        }

        // Resolve overlap by pushing particles apart
        float overlap = (minDist - distance) / 2.0f;
        float normX = dx / distance;
        float normY = dy / distance;

        particles[i].position.x -= overlap * normX;
        particles[i].position.y -= overlap * normY;
        particles[j].position.x += overlap * normX;
        particles[j].position.y += overlap * normY;

        // Relative velocity in normal direction
        float relativeVelX = particles[j].velocity.x - particles[i].velocity.x;
        float relativeVelY = particles[j].velocity.y - particles[i].velocity.y;
        float dotProduct = (relativeVelX * normX) + (relativeVelY * normY);

        if (dotProduct > 0) return; // Prevents double collision handling

        // Inelastic collision
        float impulse = dotProduct * (DAMPING_FACTOR);

        // Apply impulse to both particles
        particles[i].velocity.x += impulse * normX;
        particles[i].velocity.y += impulse * normY;
        particles[j].velocity.x -= impulse * normX;
        particles[j].velocity.y -= impulse * normY;
    }
}

__global__ void ProcessGridCollisions(Particle* particles, GridCellGPU* grid) {

    // Calculate thread index in the grid (both block and thread index)
    int idx = blockIdx.x * blockDim.x + threadIdx.x;

    if (idx >= GRID_WIDTH * GRID_HEIGHT) return;

    int x = idx % GRID_WIDTH;
    int y = idx / GRID_WIDTH;

    GridCellGPU cell = grid[idx];
    int particleCount = cell.count;
    if (particleCount == 0) return;

    // Check for collisions 
    for (int i = 0; i < particleCount; i++) {

        // Collision within the cell
        for (int j = i + 1; j < particleCount; j++) {
            ResolveCollision(particles, cell.indices[i], cell.indices[j]);
        }
    }

    // Check neighboring cells (right & bottom) only for particles near borders

    // Check right cell
    if (x + 1 < GRID_WIDTH) {
        int rightCellIndex = x + 1 + y * GRID_WIDTH;
        GridCellGPU rightCell = grid[rightCellIndex];

        // Iterate over the particles in the current cell
        for (int i = 0; i < particleCount; i++) {
            int pAIndex = cell.indices[i];
            Particle pA = particles[pAIndex];

            // If the particle A in the current cell is near the boundary and could interact with particles in the right cell
            if (pA.position.x + pA.radius > (x + 1) * GRID_CELL_WIDTH) {

                // Iterate over particles in the right cell
                for (int j = 0; j < rightCell.count; j++) {
                    // Resolve collision between particles
                    ResolveCollision(particles, pAIndex, rightCell.indices[j]);
                }
            }
        }
    }

    // Check bottom cell
    if (y + 1 < GRID_HEIGHT) {
        int bottomCellIndex = x + (y + 1) * GRID_WIDTH;
        GridCellGPU bottomCell = grid[bottomCellIndex];

        // Iterate over the particles in the current cell
        for (int i = 0; i < particleCount; i++) {
            int pAIndex = cell.indices[i];
            Particle pA = particles[pAIndex];

            // If the particle in the current cell is near the boundary and could interact with particles in the bottom cell
            if (pA.position.y + pA.radius > (y + 1) * GRID_CELL_HEIGHT) {

                // Iterate over particles in the bottom cell
                for (int j = 0; j < bottomCell.count; j++) {
                    // Resolve collision between particles
                    ResolveCollision(particles, pAIndex, bottomCell.indices[j]);
                }
            }
        }
    }
}

__global__ void ResetGrid(GridCellGPU* grid) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= GRID_WIDTH * GRID_HEIGHT) return;

    grid[idx].count = 0;
}

__global__ void AssignParticlesToGrid(Particle* particles, GridCellGPU* grid) {
    int idx = blockIdx.x * blockDim.x + threadIdx.x;
    if (idx >= NB_PARTICLES) return;

    Particle p = particles[idx];

    int gx = (int)(p.position.x / GRID_CELL_WIDTH);
    int gy = (int)(p.position.y / GRID_CELL_HEIGHT);

    // Clamp
    gx = max(0, min(gx, GRID_WIDTH - 1));
    gy = max(0, min(gy, GRID_HEIGHT - 1));

    int cellIndex = gy * GRID_WIDTH + gx;

    // Atomically insert particle index
    int insertPos = atomicAdd(&grid[cellIndex].count, 1);
    if (insertPos < MAX_PARTICLES_PER_CELL) {
        grid[cellIndex].indices[insertPos] = idx;
    }
    // Else: silently drop the particle if cell is full (can log for debugging)
}

void UpdateSimulationCuda(Particle* particles, GridCellGPU* grid, Particle* d_particles, Obstacle* d_obstacles, GridCellGPU* d_grid) {
    // Number of threads per block (you can adjust this for optimal performance)
    int blockSize = 256;
    // Number of blocks (we round up to ensure all particles are handled)
    int numBlocksParticles = (NB_PARTICLES + blockSize - 1) / blockSize;

    // Launch the kernel
    updateParticlesKernel<<<numBlocksParticles, blockSize>>>(d_particles, d_obstacles);

    int gridCellCount = GRID_WIDTH * GRID_HEIGHT;
    int numBlocksGrid = (gridCellCount + blockSize - 1) / blockSize;
    ResetGrid<<<numBlocksGrid, blockSize>>>(d_grid);

    // Synchronize the device to make sure the kernel has completed before moving on
    cudaError_t err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA sync error: %s\n", cudaGetErrorString(err));
    }

    AssignParticlesToGrid<<<numBlocksParticles, blockSize>>>(d_particles, d_grid);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA sync error: %s\n", cudaGetErrorString(err));
    }

    ProcessGridCollisions<<<numBlocksGrid, blockSize>>>(d_particles, d_grid);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA sync error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(particles, d_particles, sizeof(Particle) * NB_PARTICLES, cudaMemcpyDeviceToHost);
    return;
}