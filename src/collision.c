#include "collision.h"
#include "constants.h"
#include "simulation.h"
#include <stdio.h>
#include <stdlib.h>
#include <raymath.h>

float vector2_distance(Vector2 a, Vector2 b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

int check_collision_circles(Vector2 center1, float radius1, Vector2 center2, float radius2) {
    return vector2_distance(center1, center2) <= (radius1 + radius2);
}

// Resolve collisions with boundaries
void ResolveBoundaryCollisions(Particle* particle) {
    if (particle->position.x <= MAX_LEFT) {
        particle->position.x = MAX_LEFT;
        particle->velocity.x *= -DAMPING_FACTOR;
    }
    if (particle->position.x >= MAX_RIGHT) {
        particle->position.x = MAX_RIGHT;
        particle->velocity.x *= -DAMPING_FACTOR;
    }
    if (particle->position.y <= MAX_TOP) {
        particle->position.y = MAX_TOP;
        particle->velocity.y *= -DAMPING_FACTOR;
    }
    if (particle->position.y >= MAX_BOTTOM) {
        particle->position.y = MAX_BOTTOM;
        particle->velocity.y *= -DAMPING_FACTOR;
    }
}

// Resolve collisions with obstacles
void ResolveObstacleCollisions(Particle* particle, Obstacle* obstacles) {
    for (int j = 0; j < NB_OBSTACLES; j++) {
        Obstacle* obstacle = &obstacles[j];

        if(check_collision_circles(particle->position, particle->radius, obstacle->position, obstacle->radius)) {
            
            float dx = particle->position.x - obstacle->position.x;
            float dy = particle->position.y - obstacle->position.y;
            float distance = sqrtf(dx * dx + dy * dy);

            // Resolve overlap by pushing the particle away from the obstacle
            float overlap = (particle->radius + obstacle->radius) - distance;
            float normX = dx / distance;
            float normY = dy / distance;

            // Push the particle outside of the obstacle
            particle->position.x += overlap * normX;
            particle->position.y += overlap * normY;

            // Reflect the particle's velocity based on the normal direction
            float dotProduct = (particle->velocity.x * normX) + (particle->velocity.y * normY);

            // Reflect velocity based on the normal direction
            particle->velocity.x -= 2 * dotProduct * normX;
            particle->velocity.y -= 2 * dotProduct * normY;

            // Apply the damping factor after collision to reduce velocity
            particle->velocity.x *= DAMPING_FACTOR;
            particle->velocity.y *= DAMPING_FACTOR;
        }
    }
}

// Resolve collision between particles
void ResolveParticleCollisions(Particle* particles) {
    for (int i = 0; i < NB_PARTICLES; i++) {
        for (int j = i + 1; j < NB_PARTICLES; j++) {

            // Distance and direction between particles
            float dx = particles[j].position.x - particles[i].position.x;
            float dy = particles[j].position.y - particles[i].position.y;
            float distance = sqrtf(dx * dx + dy * dy);
            float minDist = particles[i].radius + particles[j].radius;

            if (distance < minDist) {

                // Handle perfect overlap (distance == 0)
                if (distance == 0.0f) {
                    dx = 0.005f;
                    dy = 0.005f;
                    distance = sqrtf(dx * dx + dy * dy);
                }

                // Resolve overlap by pushing particles apart
                float overlap = (minDist - distance) / 2.0f;
                float normX = dx / distance;
                float normY = dy / distance;

                particles[i].position.x -= overlap * normX;
                particles[i].position.y -= overlap * normY;
                particles[j].position.x += overlap * normX;
                particles[j].position.y += overlap * normY;

                // Relative velocity in normal direction
                float relativeVelX = particles[j].velocity.x - particles[i].velocity.x;
                float relativeVelY = particles[j].velocity.y - particles[i].velocity.y;
                float dotProduct = (relativeVelX * normX) + (relativeVelY * normY);

                if (dotProduct > 0) continue; // Prevents double collision handling

                // Inelastic collision
                float impulse = dotProduct * (DAMPING_FACTOR);

                // Apply impulse to both particles
                particles[i].velocity.x += impulse * normX;
                particles[i].velocity.y += impulse * normY;
                particles[j].velocity.x -= impulse * normX;
                particles[j].velocity.y -= impulse * normY;
            }
        }
    }
}

// Resolve collision between particles
void ResolveParticleCollision(Particle* particle1, Particle* particle2) {
    // Distance and direction between particles
    float dx = particle2->position.x - particle1->position.x;
    float dy = particle2->position.y - particle1->position.y;
    float distance = sqrtf(dx * dx + dy * dy);
    float minDist = particle1->radius + particle2->radius;

    if (distance < minDist) {
        // Handle perfect overlap (distance == 0)
        if (distance == 0.0f) {
            dx = (float)(rand() % 2 ? 1 : -1) * 0.01f; // Small random nudge
            dy = (float)(rand() % 2 ? 1 : -1) * 0.01f;
            distance = sqrtf(dx * dx + dy * dy);
        }

        // Resolve overlap by pushing particles apart
        float overlap = (minDist - distance) / 2.0f;
        float normX = dx / distance;
        float normY = dy / distance;

        particle1->position.x -= overlap * normX;
        particle1->position.y -= overlap * normY;
        particle2->position.x += overlap * normX;
        particle2->position.y += overlap * normY;

        // Relative velocity in normal direction
        float relativeVelX = particle2->velocity.x - particle1->velocity.x;
        float relativeVelY = particle2->velocity.y - particle1->velocity.y;
        float dotProduct = (relativeVelX * normX) + (relativeVelY * normY);

        if (dotProduct > 0) return; // Prevents double collision handling

        // Inelastic collision
        float impulse = dotProduct * DAMPING_FACTOR;

        // Apply impulse to both particles
        particle1->velocity.x += impulse * normX;
        particle1->velocity.y += impulse * normY;
        particle2->velocity.x -= impulse * normX;
        particle2->velocity.y -= impulse * normY;
    }
}