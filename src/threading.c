#include "threading.h"
#include "simulation.h"
#include "constants.h"

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