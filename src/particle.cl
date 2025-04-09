typedef struct {
	float x;
	float y;
} Vector2;

typedef struct {
    Vector2 position;
    Vector2 velocity;
    float radius;
    float padding;
} Particle;

typedef struct {
    Vector2 position;
	float radius;
} Obstacle;

typedef struct {
    int count;
    int indices[256];
} GridCellCL;

float vector2_distance(Vector2 a, Vector2 b) {
    float dx = b.x - a.x;
    float dy = b.y - a.y;
    return sqrt(dx * dx + dy * dy);
}

int check_collision_circles(Vector2 center1, float radius1, Vector2 center2, float radius2) {
    return vector2_distance(center1, center2) <= (radius1 + radius2);
}

__kernel void UpdateParticles(__global Particle* particles, __global Obstacle* obstacles, 
                              const float GRAVITY, const float MAX_LEFT,
							  const float MAX_RIGHT, const float MAX_TOP,
							  const float MAX_BOTTOM, const float DAMPING_FACTOR,
                              const int NUM_OBSTACLES) {

    // Get the global ID of the current thread and retrieve the particle
    int i = get_global_id(0);
    Particle* particle = &particles[i];

    // Update particle velocity with gravity and position
    particle->velocity.y += GRAVITY;
    particle->position.x += particle->velocity.x;
    particle->position.y += particle->velocity.y;

    // Resolve collisions with boundaries
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

    // Resolve collisions with obstacles
    for (int j = 0; j < NUM_OBSTACLES; j++) {
        Obstacle* obstacle = &obstacles[j];

        if (check_collision_circles(particle->position, particle->radius, obstacle->position, obstacle->radius)) {

            float dx = particle->position.x - obstacle->position.x;
            float dy = particle->position.y - obstacle->position.y;
            float distance = sqrt(dx * dx + dy * dy);

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

void ResolveCollision(__global Particle* particles, int i, int j, float DAMPING_FACTOR) {
    // Distance and direction between particles
    float dx = particles[j].position.x - particles[i].position.x;
    float dy = particles[j].position.y - particles[i].position.y;
    float distance = sqrt(dx * dx + dy * dy);
    float minDist = particles[i].radius + particles[j].radius;

    if (distance < minDist) {

        // Handle perfect overlap
        if (distance < 1e-6f) {
            dx = 0.01f;
            dy = 0.01f;
            distance = sqrt(dx * dx + dy * dy);
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

        if (dotProduct > 0) return; // Prevents double collision handling

        // Inelastic collision
        float impulse = dotProduct * (DAMPING_FACTOR);

        // Apply impulse to both particles
        particles[i].velocity.x += impulse * normX;
        particles[i].velocity.y += impulse * normY;
        particles[j].velocity.x -= impulse * normX;
        particles[j].velocity.y -= impulse * normY;
    }
}

__kernel void ProcessGridCollisions(__global Particle* particles,
                                    __global GridCellCL* grid,
                                    const int gridWidth,
                                    const int gridHeight,
                                    const float DAMPING_FACTOR) {

    int cellId = get_global_id(0);
    int x = cellId % gridWidth;
    int y = cellId / gridWidth;

    GridCellCL cell = grid[cellId];
    int particleCount = cell.count;
    if (particleCount == 0) return;
    // Iterate over particles in the cell
    for (int i = 0; i < particleCount; ++i) {
		int particleIndexA = cell.indices[i];
		Particle pA = particles[particleIndexA];

		// Check for collisions with other particles in the same cell
		for (int j = i + 1; j < particleCount; ++j) {
			int particleIndexB = cell.indices[j];
			Particle pB = particles[particleIndexB];

			// Call your collision logic here between pA and pB
			ResolveCollision(particles, particleIndexA, particleIndexB, DAMPING_FACTOR);
		}

        // Check for collisions with neighboring cells
        for (int dx = 0; dx <= 1; dx++) {
            for (int dy = 0; dy <= 1; dy++) {
                if (dx == 0 && dy == 0) continue; // skip self
				int neighborX = x + dx;
				int neighborY = y + dy;

				// Check if the neighbor cell is within bounds
				if (neighborX >= 0 && neighborX < gridWidth && neighborY >= 0 && neighborY < gridHeight) {
					int neighborCellId = neighborY * gridWidth + neighborX;
					GridCellCL neighborCell = grid[neighborCellId];

					// Iterate over particles in the neighboring cell
					for (int k = 0; k < neighborCell.count; ++k) {
						int particleIndexB = neighborCell.indices[k];
						Particle pB = particles[particleIndexB];

						// Call your collision logic here between pA and pB
						ResolveCollision(particles, particleIndexA, particleIndexB, DAMPING_FACTOR);
					}
				}
			}
		}
	}
}


