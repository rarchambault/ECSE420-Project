#include "collision.h"
#include <raymath.h>

// Simple distance-based collision resolution
void ResolveCollisions(Particle* particles, int count) {
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            float dx = particles[j].position.x - particles[i].position.x;
            float dy = particles[j].position.y - particles[i].position.y;
            float distance = sqrtf(dx * dx + dy * dy);
            float minDist = particles[i].radius + particles[j].radius;

            if (distance < minDist) {
                // Resolve overlap by pushing particles apart
                float overlap = (minDist - distance) / 2;
                particles[i].position.x -= overlap * (dx / distance);
                particles[i].position.y -= overlap * (dy / distance);
                particles[j].position.x += overlap * (dx / distance);
                particles[j].position.y += overlap * (dy / distance);

                // Swap velocities (simple elastic collision)
                Vector2 temp = particles[i].velocity;
                particles[i].velocity = particles[j].velocity;
                particles[j].velocity = temp;
            }
        }
    }
}
