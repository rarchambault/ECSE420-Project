#include "renderer.h"
#include "simulation.h"
#include "raylib.h"

void InitRenderer() {}

void RenderParticles() {
    Particle* particles = GetParticles();
    for (int i = 0; i < MAX_PARTICLES; i++) {
        DrawCircleV(particles[i].position, 5, (Color) { 203, 66, 245, 255 });
    }
}

void CleanupRenderer() {}