#ifndef COLLISION_H
#define COLLISION_H

#include "raylib.h"

typedef struct {
    Vector2 position;
    Vector2 velocity;
    float radius;
    float _padding; // Padding to align the structure size to 24 bytes
} Particle;

typedef struct {
    Vector2 position;
    float radius;
} Obstacle;


float vector2_distance(Vector2 a, Vector2 b);
int check_collision_circles(Vector2 center1, float radius1, Vector2 center2, float radius2);
void ResolveParticleCollisions(Particle* particles);
void ResolveParticleCollision(Particle* particle1, Particle* particle2);
void ResolveBoundaryCollisions(Particle* particles);
void ResolveObstacleCollisions(Particle* particles, Obstacle* obstacles);

#endif
