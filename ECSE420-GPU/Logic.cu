#include "Render.cuh"
#include "Constants.cuh"
#include "Logic.cuh"
#include "Collisions.cuh"  // Include the collision module.
#include <cuda_runtime.h>
#include <device_launch_parameters.h>
#include <math.h>


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
            framebuffer[pixelIdx + 0] = 255; // dark gray
            framebuffer[pixelIdx + 1] = 0;
            framebuffer[pixelIdx + 2] = 0;
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
            framebuffer[pixelIdx + 0] = 0;
            framebuffer[pixelIdx + 1] = 0;
            framebuffer[pixelIdx + 2] = 255;
            break;
        }
    }
}



/// SIMULATION ///



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



void runStep(Particle* d_particles[], int numParticles, float deltaTime, float gravity,
    int* d_gridCounters, int* d_gridIndices, Obstacle* d_obstacles,
    int numObstacles, unsigned char* d_framebuffer, int width, int height, int frameNumber, cudaStream_t simStream, cudaStream_t renderStream, cudaEvent_t simEvent,
    int simBuffer, int renderBuffer) {

            int threadsPerBlock_sim = 256;
            int numBlocks_sim = (numParticles + threadsPerBlock_sim - 1) / threadsPerBlock_sim;

            dim3 threadsPerBlock_render(16, 16);
            dim3 numBlocks_render((width + 15) / 16, (height + 15) / 16);

            // 2. Reset the grid counters.
            int numCells = GRID_WIDTH * GRID_HEIGHT;

            cudaMemset(d_gridCounters, 0, numCells * sizeof(int));

            // 1. Integration: update particle positions and velocities.
            integrateParticlesKernel << <numBlocks_sim, threadsPerBlock_sim, 0, simStream>> >
                (d_particles[simBuffer], numParticles, deltaTime, gravity, d_obstacles, numObstacles);

            // 3. Build the uniform grid.
            buildGridKernel << <numBlocks_sim, threadsPerBlock_sim, 0, simStream>> > (d_particles[simBuffer], numParticles, d_gridCounters, d_gridIndices);

            // 4. Resolve collisions using the dedicated collision module.
            resolveCollisionsKernelSymmetric << <numBlocks_sim, threadsPerBlock_sim, 0, simStream>> > (d_particles[simBuffer], d_gridCounters, d_gridIndices, numParticles);

            cudaEventRecord(simEvent, simStream);
            
            
            if (frameNumber > 0) {
                cudaStreamWaitEvent(renderStream, simEvent, 0);

                renderParticlesKernel << <numBlocks_render, threadsPerBlock_render, 0, renderStream >> > (
                    d_particles[renderBuffer], numParticles,
                    d_obstacles, numObstacles,
                    d_framebuffer, width, height, frameNumber);
            }

}
