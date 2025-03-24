#include <stdlib.h>
#include "simulation.h"
#include "constants.h"


static Particle particles[MAX_PARTICLES];

void InitSimulation() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].position = (Vector2){
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_WIDTH - 2 * PARTICLE_RADIUS),
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_HEIGHT - 2 * PARTICLE_RADIUS)
        };
        particles[i].velocity = (Vector2){
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f,  // Random float between -1.0 and 1.0
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f
        };
        particles[i].radius = PARTICLE_RADIUS;
    }
}

void UpdateSimulation() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;

        // Walls
        if (particles[i].position.x <= MAX_LEFT) {
            particles[i].position.x = MAX_LEFT;
            particles[i].velocity.x *= -1;
        }
        if (particles[i].position.x >= MAX_RIGHT) {
            particles[i].position.x = MAX_RIGHT;
            particles[i].velocity.x *= -1;
        }
        if (particles[i].position.y <= MAX_TOP) {
            particles[i].position.y = MAX_TOP;
            particles[i].velocity.y *= -1;
        }
        if (particles[i].position.y >= MAX_BOTTOM) {
            particles[i].position.y = MAX_BOTTOM;
            particles[i].velocity.y *= -1;
        }
    }

    // Call collision handling
    ResolveCollisions(particles, MAX_PARTICLES);
}

Particle* GetParticles() {
    return particles;
}

void CleanupSimulation() {}
