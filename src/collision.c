#include "collision.h"
#include "constants.h"
#include <raymath.h>

// Simple distance-based collision resolution
void ResolveCollisions(Particle* particles, int count) {
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {

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
