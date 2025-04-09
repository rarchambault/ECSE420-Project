#ifndef THREADNG_H
#define THREADING_H

typedef struct {
    int threadIndex;
} CpuThreadData;

void AssignParticlesToGrid();
void* ProcessBoundariesAndObstacles(void* arg);
void* ProcessParticleCollisions(void* args);

#endif