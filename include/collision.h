#ifndef COLLISION_H
#define COLLISION_H

#include "raylib.h"

typedef struct {
    Vector2 position;
    Vector2 velocity;
    float radius;
} Particle;

void ResolveCollisions(Particle* particles, int count);

#endif
