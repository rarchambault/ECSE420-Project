#include "Simulation.cuh"
#include "Collisions.cuh"  // Include the collision module.
#include "Constants.cuh"
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>

// Integration kernel: Updates particle velocity (with gravity) and position.
__global__ void integrateParticlesKernel(Particle* particles, int numParticles, float deltaTime, float gravity,
    Obstacle* obstacles, int numObstacles) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numParticles) return;

    Particle p = particles[i];

    // Gravity
    p.velocity.y += gravity * deltaTime;

    // Integrate motion
    p.position.x += p.velocity.x * deltaTime;
    p.position.y += p.velocity.y * deltaTime;

    // Obstacle collisions
    for (int o = 0; o < numObstacles; o++) {
        Obstacle obs = obstacles[o];
        float dx = p.position.x - obs.position.x;
        float dy = p.position.y - obs.position.y;
        float dist = sqrtf(dx * dx + dy * dy);
        float minDist = p.radius + obs.radius;

        if (dist < minDist && dist > 0.0f) {
            float normX = dx / dist;
            float normY = dy / dist;
            float overlap = minDist - dist;

            p.position.x += normX * overlap;
            p.position.y += normY * overlap;

            float vDotN = p.velocity.x * normX + p.velocity.y * normY;
            if (vDotN < 0.0f) {
                p.velocity.x -= 2.0f * vDotN * normX;
                p.velocity.y -= 2.0f * vDotN * normY;
            }

            p.velocity.x *= DAMPING_FACTOR;
            p.velocity.y *= DAMPING_FACTOR;
        }
    }

    // Boundary collision
    if (p.position.x < MAX_LEFT) {
        p.position.x = MAX_LEFT;
        p.velocity.x = -p.velocity.x * DAMPING_FACTOR;
    }
    if (p.position.x > MAX_RIGHT) {
        p.position.x = MAX_RIGHT;
        p.velocity.x = -p.velocity.x * DAMPING_FACTOR;
    }
    if (p.position.y < MAX_TOP) {
        p.position.y = MAX_TOP;
        p.velocity.y = -p.velocity.y * DAMPING_FACTOR;
    }
    if (p.position.y > MAX_BOTTOM) {
        p.position.y = MAX_BOTTOM;
        p.velocity.y = -p.velocity.y * DAMPING_FACTOR;
    }

    particles[i] = p;
}

// Grid building kernel: Bins particles into a uniform grid using atomic operations.
__global__ void buildGridKernel(Particle* particles, int numParticles,
    int* gridCounters, int* gridIndices) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i >= numParticles) return;

    Particle p = particles[i];
    int cellX = (int)(p.position.x / GRID_CELL_SIZE);
    int cellY = (int)(p.position.y / GRID_CELL_SIZE);

    cellX = min(max(cellX, 0), static_cast<int>(GRID_WIDTH) - 1);
    cellY = min(max(cellY, 0), static_cast<int>(GRID_HEIGHT) - 1);


    int cellIndex = cellY * GRID_WIDTH + cellX;

    int offset = atomicAdd(&gridCounters[cellIndex], 1);
    if (offset < MAX_PARTICLES_PER_CELL) {
        gridIndices[cellIndex * MAX_PARTICLES_PER_CELL + offset] = i;
    }
}

// Host function that performs one simulation step.
void runSimulationStep(Particle* d_particles, int numParticles, float deltaTime, float gravity,
    int* d_gridCounters, int* d_gridIndices, Obstacle* obstacles,
    int numObstacles) {
    int threadsPerBlock = 256;
    int numBlocks = (numParticles + threadsPerBlock - 1) / threadsPerBlock;

    // 1. Integration: update particle positions and velocities.
    integrateParticlesKernel << <numBlocks, threadsPerBlock >> > (
        d_particles, numParticles, deltaTime, gravity, obstacles, numObstacles);

    // 2. Reset the grid counters.
    int numCells = GRID_WIDTH * GRID_HEIGHT;
    cudaMemset(d_gridCounters, 0, numCells * sizeof(int));

    // 3. Build the uniform grid.
    buildGridKernel << <numBlocks, threadsPerBlock >> > (d_particles, numParticles, d_gridCounters, d_gridIndices);

    // 4. Resolve collisions using the dedicated collision module.
    runCollisionStep(d_particles, d_gridCounters, d_gridIndices, numParticles);
}

