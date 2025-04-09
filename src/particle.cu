#include "particle.h"
#include "simulation.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <cuda.h>
#include <cuda_runtime.h>
#include <math.h>

__device__ float vector2_distance(Vector2 a, Vector2 b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

__device__ int check_collision_circles(Vector2 center1, float radius1, Vector2 center2, float radius2) {
    return vector2_distance(center1, center2) <= (radius1 + radius2);
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

        if (check_collision_circles(particle->position, particle->radius, obstacle->position, obstacle->radius)) {

            float dx = particle->position.x - obstacle->position.x;
            float dy = particle->position.y - obstacle->position.y;
            float distance = sqrt(dx * dx + dy * dy);

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

    // Ensure that we don't go out of bounds
    if (idx >= GRID_WIDTH * GRID_HEIGHT) return;

    // Calculate x, y coordinates of the grid cell from the thread index
    int x = idx % GRID_WIDTH;
    int y = idx / GRID_HEIGHT;

    // Access the grid cell
    GridCellGPU* cell = &grid[idx];

    int particleCount = cell->count;
    if (particleCount == 0) return;

    // Iterate over particles in the cell
    for (int i = 0; i < particleCount; ++i) {
        int particleIndexA = cell->indices[i];

        // Check for collisions with other particles in the same cell
        for (int j = i + 1; j < particleCount; ++j) {
            int particleIndexB = cell->indices[j];

            // Call your collision logic here between pA and pB
            ResolveCollision(particles, particleIndexA, particleIndexB);
        }

        // Check for collisions with neighboring cells
        for (int dx = 0; dx <= 1; dx++) {
            for (int dy = 0; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue; // skip self
                int neighborX = x + dx;
                int neighborY = y + dy;

                // Check if the neighbor cell is within bounds
                if (neighborX >= 0 && neighborX < GRID_WIDTH && neighborY >= 0 && neighborY < GRID_HEIGHT) {
                    int neighborCellId = neighborY * GRID_WIDTH + neighborX;
                    GridCellGPU neighborCell = grid[neighborCellId];

                    // Iterate over particles in the neighboring cell
                    for (int k = 0; k < neighborCell.count; ++k) {
                        int particleIndexB = neighborCell.indices[k];

                        // Call your collision logic here between pA and pB
                        ResolveCollision(particles, particleIndexA, particleIndexB);
                    }
                }
            }
        }
    }
}

void UpdateSimulationCuda(Particle* particles, GridCellGPU* grid, Particle* d_particles, Obstacle* d_obstacles, GridCellGPU* d_grid) {
    // Number of threads per block (you can adjust this for optimal performance)
    int blockSize = 256;
    // Number of blocks (we round up to ensure all particles are handled)
    int numBlocks = (NB_PARTICLES + blockSize - 1) / blockSize;

    // Launch the kernel
    updateParticlesKernel<<<numBlocks, blockSize>>>(d_particles, d_obstacles);

    // Check for errors during kernel launch
    cudaError_t err = cudaGetLastError();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA error: %s\n", cudaGetErrorString(err));
        return;
    }

    // Synchronize the device to make sure the kernel has completed before moving on
    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA sync error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(particles, d_particles, sizeof(Particle) * NB_PARTICLES, cudaMemcpyDeviceToHost);

    // Handle particle collisions
    // Reset the grid cell counts to zero
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            GridCellGPU* cell = grid + (y * GRID_WIDTH + x);
            cell->count = 0;
        }
    }

    // Assign particles to grid cells
    for (int i = 0; i < NB_PARTICLES; i++) {
        int gx = particles[i].position.x / GRID_CELL_WIDTH;
        int gy = particles[i].position.y / GRID_CELL_HEIGHT;

        if (gx < 0) gx = 0;
        if (gx >= GRID_WIDTH) gx = GRID_WIDTH - 1;
        if (gy < 0) gy = 0;
        if (gy >= GRID_HEIGHT) gy = GRID_HEIGHT - 1;

        GridCellGPU* cell = grid + (gy * GRID_WIDTH + gx);
        if (cell->count < MAX_PARTICLES_PER_CELL) {
            cell->indices[cell->count++] = i;
        }
    }

    // Copy grid to device
    err = cudaMemcpy(d_grid, grid, sizeof(GridCellGPU) * GRID_WIDTH * GRID_HEIGHT, cudaMemcpyHostToDevice);
    if (err != cudaSuccess) {
		fprintf(stderr, "CUDA memcpy error: %s\n", cudaGetErrorString(err));
		return;
	}

    blockSize = 128;
    numBlocks = (GRID_WIDTH * GRID_HEIGHT + blockSize - 1) / blockSize;
    ProcessGridCollisions<<<numBlocks, blockSize>>>(d_particles, d_grid);

    err = cudaDeviceSynchronize();
    if (err != cudaSuccess) {
        fprintf(stderr, "CUDA sync error: %s\n", cudaGetErrorString(err));
    }

    cudaMemcpy(particles, d_particles, sizeof(Particle) * NB_PARTICLES, cudaMemcpyDeviceToHost);
    return;
}