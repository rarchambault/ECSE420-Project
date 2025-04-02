#include "Simulation.cuh"
#include "Render.cuh"
#include "Constants.cuh"
#include "Collisions.cuh"
#include <cuda_runtime.h>
#include <iostream>
#include <fstream>
#include <cstdlib>
//#include "raylib.h"
#include "../build/external/raylib-master/src/raylib.h"

void saveFramePPM(const char* filename, unsigned char* framebuffer, int width, int height) {
    std::ofstream ofs(filename, std::ios::binary);
    if (!ofs) {
        std::cerr << "Error: Could not open file " << filename << " for writing." << std::endl;
        return;
    }
    // Write the PPM header: P6, width, height, and max color value.
    ofs << "P6\n" << width << " " << height << "\n255\n";
    // Write pixel data: for each pixel, write only the first 3 bytes (RGB)
    for (int i = 0; i < width * height; i++) {
        ofs.write(reinterpret_cast<char*>(&framebuffer[i * 4]), 3);
    }
    ofs.close();
}

int main() {
    // Set the number of particles.
    const int numParticles = 128;
    size_t particlesSize = numParticles * sizeof(Particle);

    // Allocate and initialize host particle data.
    Particle* h_particles = new Particle[numParticles];
    for (int i = 0; i < numParticles; i++) {
        h_particles[i].position = make_float2(rand() % WINDOW_WIDTH, rand() % WINDOW_HEIGHT);
        h_particles[i].velocity = make_float2(((rand() % 200) / 100.0f) - 1.0f,
            ((rand() % 200) / 100.0f) - 1.0f);
        h_particles[i].radius = PARTICLE_RADIUS;
    }

    Obstacle h_obstacles[] = {
    { make_float2(400.0f, 300.0f), 40.0f },
    { make_float2(200.0f, 500.0f), 25.0f }
    };
    int numObstacles = sizeof(h_obstacles) / sizeof(Obstacle);

    Obstacle* d_obstacles;
    cudaMalloc(&d_obstacles, sizeof(h_obstacles));
    cudaMemcpy(d_obstacles, h_obstacles, sizeof(h_obstacles), cudaMemcpyHostToDevice);

    // Allocate device memory for particles.
    Particle* d_particles;
    cudaMalloc(&d_particles, particlesSize);
    cudaMemcpy(d_particles, h_particles, particlesSize, cudaMemcpyHostToDevice);

    // Allocate device memory for grid arrays.
    int numCells = GRID_WIDTH * GRID_HEIGHT;
    int* d_gridCounters, * d_gridIndices;
    cudaMalloc(&d_gridCounters, numCells * sizeof(int));
    cudaMalloc(&d_gridIndices, numCells * MAX_PARTICLES_PER_CELL * sizeof(int));

    // Allocate device framebuffer (RGBA) for rendering.
    int width = WINDOW_WIDTH, height = WINDOW_HEIGHT;
    size_t framebufferSize = width * height * 4 * sizeof(unsigned char);
    unsigned char* d_framebuffer;
    cudaMalloc(&d_framebuffer, framebufferSize);

    // Allocate host memory for the framebuffer.
    unsigned char* h_framebuffer = new unsigned char[framebufferSize];

    // Simulation loop: Render and save a sequence of frames.
    const int numFrames = 1000;
    
    //SetConfigFlags(FLAG_VSYNC_HINT | FLAG_WINDOW_HIGHDPI);
    InitWindow(WINDOW_WIDTH, WINDOW_HEIGHT, "Particle Simulator");

    Image image;
    image.data = h_framebuffer;
    image.width = WINDOW_WIDTH;
    image.height = WINDOW_HEIGHT;
    image.mipmaps = 1;
    image.format = PIXELFORMAT_UNCOMPRESSED_R8G8B8A8;
    Texture2D texture = LoadTextureFromImage(image);

    for (int frame = 0; frame < numFrames; frame++) {
        float deltaTime = 0.016f; // ~60 fps

        // Run one simulation step: integration, grid building, and collision resolution.
        runSimulationStep(d_particles, numParticles, deltaTime, GRAVITY, d_gridCounters, d_gridIndices, d_obstacles, numObstacles);

        // Render particles into the device framebuffer.
        renderParticles(d_particles, numParticles,
            d_obstacles, numObstacles,
            d_framebuffer, width, height, frame);

        // Copy the framebuffer from device to host.
        cudaMemcpy(h_framebuffer, d_framebuffer, framebufferSize, cudaMemcpyDeviceToHost);

        // Display current framebuffer
        UpdateTexture(texture, h_framebuffer);

        BeginDrawing();
        ClearBackground(RAYWHITE);
        DrawTexture(texture, 0, 0, WHITE);
        EndDrawing();
    }

    // Clean up Raylib texture.
    UnloadTexture(texture);

    // Clean up CUDA memory.
    cudaFree(d_particles);
    cudaFree(d_gridCounters);
    cudaFree(d_gridIndices);
    cudaFree(d_framebuffer);

    // Clean up host memory.
    delete[] h_particles;
    delete[] h_framebuffer;

    return 0;
}