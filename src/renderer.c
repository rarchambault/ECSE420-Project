#include "renderer.h"
#include "simulation.h"
#include "raylib.h"


static void RenderParticles() {
	Particle* particles = GetParticles();
	for (int i = 0; i < NB_PARTICLES; i++) {
		DrawCircleV(particles[i].position, particles[i].radius, BLUE);
	}
}

static void RenderObstacles() {
	Obstacle* obstacles = GetObstacles();
	for (int i = 0; i < NB_OBSTACLES; i++) {
		DrawCircleV(obstacles[i].position, obstacles[i].radius, RED);
	}
}

void InitRenderer() {}

void Render() {
	BeginDrawing();
	ClearBackground(BLACK);
	RenderParticles();
	RenderObstacles();
	EndDrawing();
}

void CleanupRenderer() {}