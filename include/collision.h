#ifndef COLLISION_H
#define COLLISION_H

#include "raylib.h"

typedef struct {
    Vector2 position;
    Vector2 velocity;
    float radius;
} Particle;

typedef struct {
    Vector2 position;
    float radius;
} Obstacle;

void ResolveParticleCollisions(Particle* particles);
void ResolveParticleCollision(Particle* particle1, Particle* particle2);
void ResolveBoundaryCollisions(Particle* particles);
void ResolveObstacleCollisions(Particle* particles, Obstacle* obstacles);

#endif
