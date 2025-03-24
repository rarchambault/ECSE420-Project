#include "collision.h"
#include "constants.h"
#include "simulation.h"
#include <raymath.h>

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

        if(CheckCollisionCircles(particle->position, particle->radius, obstacle->position, obstacle->radius)) {
            
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
                    dx = (float)(rand() % 2 ? 1 : -1) * 0.01f; // Small random nudge
                    dy = (float)(rand() % 2 ? 1 : -1) * 0.01f;
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
