#ifndef SIMULATION_H
#define SIMULATION_H

#include "collision.h"

#define MAX_PARTICLES 100

void InitSimulation();
void UpdateSimulation();
void CleanupSimulation();
Particle* GetParticles();

#endif