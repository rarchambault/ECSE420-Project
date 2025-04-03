#include "simulation.h"
#include "constants.h"
#include <stdlib.h>
#include <pthread.h>

pthread_t threads[NUM_THREADS_CPU];

typedef struct {
    int threadIndex;
} CpuThreadData;

CpuThreadData threadData[NUM_THREADS_CPU];

typedef struct {
    int count;
    Particle* particles[NB_PARTICLES]; // Stores pointers to particles in this cell
} GridCell;

static GridCell grid[GRID_WIDTH][GRID_HEIGHT]; // The simulation grid
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
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_HEIGHT / 2 - 2 * PARTICLE_RADIUS)
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
void AssignParticlesToGrid() {
    // Step 1: Reset the grid cell counts to zero
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            grid[x][y].count = 0;  // Reset count of particles in each cell
        }
    }

    // Step 2: Assign particles to their respective grid cells
    for (int i = 0; i < NB_PARTICLES; i++) {
        int x = (int)(particles[i].position.x / GRID_CELL_WIDTH);
        int y = (int)(particles[i].position.y / GRID_CELL_HEIGHT);

        // Ensure x and y are within the grid's bounds
        if (x >= GRID_WIDTH) x = GRID_WIDTH - 1;
        if (y >= GRID_HEIGHT) y = GRID_HEIGHT - 1;

        int count = grid[x][y].count;
        grid[x][y].particles[count] = &particles[i];  // Append particle
        grid[x][y].count++;  // Increase count
    }
}

void* ProcessBoundariesAndObstacles(void* arg) {
    CpuThreadData* data = (CpuThreadData*)arg;

    for (int i = data->threadIndex; i < NB_PARTICLES; i += NUM_THREADS_CPU) {
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

    for (int cellIndex = data->threadIndex; cellIndex < GRID_WIDTH * GRID_HEIGHT; cellIndex += NUM_THREADS_CPU) {
        int x = cellIndex % GRID_WIDTH;
        int y = cellIndex / GRID_WIDTH;

        // Skip empty cells (no particles)
        if (grid[x][y].count == 0) continue;

        GridCell* cell = &grid[x][y];

        // Check collisions within the cell
        for (int i = 0; i < cell->count; i++) {
            for (int j = i + 1; j < cell->count; j++) {
                ResolveParticleCollision(cell->particles[i], cell->particles[j]);
            }
        }

        // Check neighboring cells (right & bottom) only for particles near borders
        if (x + 1 < GRID_WIDTH && grid[x + 1][y].count > 0) {
            GridCell* right = &grid[x + 1][y];
            for (int i = 0; i < cell->count; i++) {
                if (cell->particles[i]->position.x + cell->particles[i]->radius > (x + 1) * GRID_CELL_WIDTH) {
                    for (int j = 0; j < right->count; j++) {
                        ResolveParticleCollision(cell->particles[i], right->particles[j]);
                    }
                }
            }
        }

        if (y + 1 < GRID_HEIGHT && grid[x][y + 1].count > 0) {
            GridCell* below = &grid[x][y + 1];
            for (int i = 0; i < cell->count; i++) {
                if (cell->particles[i]->position.y + cell->particles[i]->radius > (y + 1) * GRID_CELL_HEIGHT) {
                    for (int j = 0; j < below->count; j++) {
                        ResolveParticleCollision(cell->particles[i], below->particles[j]);
                    }
                }
            }
        }
    }

    return NULL;
}

void UpdateSimulation_CpuThreading() {
    // Step 1: Assign particles to the grid
    AssignParticlesToGrid();

    // Step 2: Handle boundary & obstacle collisions
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        threadData[i].threadIndex = i;
        pthread_create(&threads[i], NULL, ProcessBoundariesAndObstacles, &threadData[i]);
    }
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_join(threads[i], NULL);
    }

    // Step 3: Handle particle collisions per grid cell
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        threadData[i].threadIndex = i;
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