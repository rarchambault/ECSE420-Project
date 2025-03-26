#include "simulation.h"
#include "constants.h"
#include <stdlib.h>


static Particle particles[NB_PARTICLES];
static Obstacle obstacles[NB_OBSTACLES] = {
    {100, 150, 30.0f},
    {400, 300, 50.0f},
	{200, 400, 20.0f},
    {600, 200, 40.0f},
	{700, 500, 60.0f},
    {800, 100, 10.0f}
};

void InitSimulation() {
    for (int i = 0; i < NB_PARTICLES; i++) {
        particles[i].position = (Vector2){
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_WIDTH - 2 * PARTICLE_RADIUS),
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_HEIGHT/2 - 2 * PARTICLE_RADIUS)
        };
        particles[i].velocity = (Vector2){
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f,  // Random float between -1.0 and 1.0
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f
        };
        particles[i].radius = PARTICLE_RADIUS;
    }
}

void UpdateSimulation() {
    for (int i = 0; i < NB_PARTICLES; i++) {
        // Update velocity (gravity)
        particles[i].velocity.y += GRAVITY;

        // Update position
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;

        // Resolve collisions with walls
        ResolveBoundaryCollisions(&particles[i]);

        // Resolve collisions with obstacles
        ResolveObstacleCollisions(&particles[i], obstacles);
    }

    // Resolve collisions between particles
    ResolveParticleCollisions(particles);
}

Particle* GetParticles() {
    return particles;
}

Obstacle* GetObstacles() {
	return obstacles;
}

void CleanupSimulation() {}
