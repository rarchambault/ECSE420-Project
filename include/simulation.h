#ifndef SIMULATION_H
#define SIMULATION_H

#include "collision.h"

#define NB_PARTICLES 500
#define NB_OBSTACLES 6

void InitSimulation();
void UpdateSimulation();
void CleanupSimulation();
Particle* GetParticles();
Obstacle* GetObstacles();

#endif