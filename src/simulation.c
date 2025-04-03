#include "simulation.h"
#include "constants.h"
#include <stdlib.h>
#include <pthread.h>

pthread_t threads[NUM_THREADS_CPU];
CpuThreadData threadData[NUM_THREADS_CPU];

static Particle particles[NB_PARTICLES];
static Obstacle obstacles[NB_OBSTACLES] = {
    {100, 150, 30.0f},
    {400, 300, 50.0f},
	{200, 400, 20.0f},
    {600, 200, 40.0f},
	{700, 500, 60.0f},
    {800, 100, 10.0f}
};

void InitSimulation() {
    for (int i = 0; i < NB_PARTICLES; i++) {
        particles[i].position = (Vector2){
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_WIDTH - 2 * PARTICLE_RADIUS),
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_HEIGHT/2 - 2 * PARTICLE_RADIUS)
        };
        particles[i].velocity = (Vector2){
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f,  // Random float between -1.0 and 1.0
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f
        };
        particles[i].radius = PARTICLE_RADIUS;
    }
}

// Sequential version
void UpdateSimulation_Sequential() {
    for (int i = 0; i < NB_PARTICLES; i++) {
        particles[i].velocity.y += GRAVITY;
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;
        ResolveBoundaryCollisions(&particles[i]);
        ResolveObstacleCollisions(&particles[i], obstacles);
    }
    ResolveParticleCollisions(particles);
}

// CPU threading version
void* ProcessBoundariesAndObstacles(void* arg) {
    CpuThreadData* data = (CpuThreadData*)arg;
    for (int i = data->start; i < data->end; i++) {
        particles[i].velocity.y += GRAVITY;
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;
        ResolveBoundaryCollisions(&particles[i]);
        ResolveObstacleCollisions(&particles[i], obstacles);
    }
    return NULL;
}

void* ProcessParticleCollisions(void* arg) {
    CpuThreadData* data = (CpuThreadData*)arg;
    for (int i = data->start; i < data->end; i++) {
        for (int j = i + 1; j < NB_PARTICLES; j++) {
            ResolveParticleCollision(&particles[i], &particles[j]);
        }
    }
    return NULL;
}

void UpdateSimulation_CpuThreading() {
    int chunkSize = NB_PARTICLES / NUM_THREADS_CPU;

    // First pass: Boundary and obstacle collisions
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        threadData[i].start = i * chunkSize;
        threadData[i].end = (i == NUM_THREADS_CPU - 1) ? NB_PARTICLES : (i + 1) * chunkSize;
        pthread_create(&threads[i], NULL, ProcessBoundariesAndObstacles, &threadData[i]);
    }
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_join(threads[i], NULL);
    }

    // Second pass: Particle collisions
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_create(&threads[i], NULL, ProcessParticleCollisions, &threadData[i]);
    }
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_join(threads[i], NULL);
    }
}


void UpdateSimulation() {
    switch (executionMode)
    {
        case EXECUTION_CPU_THREADING: 
            UpdateSimulation_CpuThreading();
            break;
        case EXECUTION_SEQUENTIAL:
            UpdateSimulation_Sequential();
            break;
        default:
            UpdateSimulation_Sequential();
            break;
    }
}

Particle* GetParticles() {
    return particles;
}

Obstacle* GetObstacles() {
	return obstacles;
}

void CleanupSimulation() {}
