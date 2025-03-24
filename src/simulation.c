#include "simulation.h"
#include <stdlib.h>

static Particle particles[MAX_PARTICLES];

void InitSimulation() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].position = (Vector2){ rand() % 1280, rand() % 800 };
        particles[i].velocity = (Vector2){ (rand() % 3) - 1.5f, (rand() % 3) - 1.5f };
        particles[i].radius = 5.0f;
    }
}

void UpdateSimulation() {
    for (int i = 0; i < MAX_PARTICLES; i++) {
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;

        // Bounce off walls
        if (particles[i].position.x <= 0 || particles[i].position.x >= 1280)
            particles[i].velocity.x *= -1;
        if (particles[i].position.y <= 0 || particles[i].position.y >= 800)
            particles[i].velocity.y *= -1;
    }

    // Call collision handling
    ResolveCollisions(particles, MAX_PARTICLES);
}

Particle* GetParticles() {
    return particles;
}

void CleanupSimulation() {}
