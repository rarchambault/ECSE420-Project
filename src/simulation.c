#include "simulation.h"
#include "constants.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <CL/cl.h>

pthread_t threads[NUM_THREADS_CPU];

typedef struct {
    int threadIndex;
} CpuThreadData;

CpuThreadData threadData[NUM_THREADS_CPU];

typedef struct {
    int count;
    Particle* particles[NB_PARTICLES]; // Stores pointers to particles in this cell
} GridCell;

typedef struct {
    int count;
    int indices[MAX_PARTICLES_PER_CELL];
} GridCellCL;

static GridCell grid[GRID_WIDTH][GRID_HEIGHT]; // The simulation grid
static Particle particles[NB_PARTICLES];
static Obstacle obstacles[NB_OBSTACLES] = {
    {100, 150, 30.0f},
    {400, 300, 50.0f},
    {200, 400, 20.0f},
    {600, 200, 40.0f},
    {700, 500, 60.0f},
    {800, 100, 10.0f}
};

cl_platform_id platform;
cl_device_id device;
cl_context context;
cl_command_queue queue;
cl_program program;
cl_kernel kernel1;
cl_kernel kernel2;
cl_int err;
cl_mem particleBuffer;
cl_mem gridBuffer;

void InitSimulation() {
    for (int i = 0; i < NB_PARTICLES; i++) {
        particles[i].position = (Vector2){
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_WIDTH - 2 * PARTICLE_RADIUS),
            PARTICLE_RADIUS + ((float)rand() / RAND_MAX) * (WINDOW_HEIGHT / 2 - 2 * PARTICLE_RADIUS)
        };
        particles[i].velocity = (Vector2){
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f,  // Random float between -1.0 and 1.0
            ((float)rand() / RAND_MAX) * 2.0f - 1.0f
        };
        particles[i].radius = PARTICLE_RADIUS;
    }

    if (executionMode == EXECUTION_OPENCL_GPU) {

        err = clGetPlatformIDs(1, &platform, NULL);
        printf("clGetPlatformIDs: %d\n", err);
        if (err != CL_SUCCESS) return;

        err = clGetDeviceIDs(platform, CL_DEVICE_TYPE_GPU, 1, &device, NULL);
        printf("clGetDeviceIDs: %d\n", err);
        if (err != CL_SUCCESS) return;

        context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
        printf("clCreateContext: %d\n", err);
        if (!context || err != CL_SUCCESS) return;

        queue = clCreateCommandQueue(context, device, 0, &err);
        printf("clCreateCommandQueue: %d\n", err);
        if (!queue || err != CL_SUCCESS) return;

        FILE* file = fopen("src/particle.cl", "rb");
        fseek(file, 0, SEEK_END);
        size_t sourceSize = ftell(file);
        rewind(file);
        char* sourceCode = (char*)malloc(sourceSize + 1);
        size_t bytesRead = fread(sourceCode, 1, sourceSize, file);
        sourceCode[sourceSize] = '\0';
        fclose(file);
        const char* sourcePtr = sourceCode;
        size_t sourceSizes[1] = { sourceSize };
        program = clCreateProgramWithSource(context, 1, &sourcePtr, sourceSizes, &err);
        printf("clCreateProgramWithSource: %d\n", err);
        if(!program || err != CL_SUCCESS) return;

        err = clBuildProgram(program, 1, &device, NULL, NULL, NULL);
        if (err != CL_SUCCESS) {
            printf("clBuildProgram failed with error code %d\n", err);

            // Get the build log
            size_t logSize;
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
            char* buildLog = (char*)malloc(logSize + 1);
            clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, logSize, buildLog, NULL);
            buildLog[logSize] = '\0';

            // Print the build log to diagnose the problem
            printf("OpenCL Build Log:\n%s\n", buildLog);

            free(buildLog);
            return;
        }

        printf("Successfully built OpenCL program.\n");

        kernel1 = clCreateKernel(program, "UpdateParticles", &err);
        printf("clCreateKernel: %d\n", err);
        if (!kernel1 || err != CL_SUCCESS) return;

        kernel2 = clCreateKernel(program, "ProcessGridCollisions", &err);
        printf("clCreateKernel: %d\n", err);
        if (!kernel2 || err != CL_SUCCESS) return;

        particleBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Particle) * NB_PARTICLES, NULL, &err);
        printf("clCreateBuffer: %d\n", err);
        if (!particleBuffer || err != CL_SUCCESS) return;

        cl_mem obstacleBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(Obstacle) * NB_OBSTACLES, NULL, &err);
        printf("clCreateBuffer: %d\n", err);
        if (!obstacleBuffer || err != CL_SUCCESS) return;

        gridBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(GridCell) * (GRID_WIDTH * GRID_HEIGHT), NULL, &err);
        printf("clCreateBuffer: %d\n", err);
        if (!gridBuffer || err != CL_SUCCESS) return;

        err = clEnqueueWriteBuffer(queue, particleBuffer, CL_TRUE, 0, sizeof(Particle) * NB_PARTICLES, particles, 0, NULL, NULL);
        printf("clEnqueueWriteBuffer: %d\n", err);
        if (err != CL_SUCCESS) return;

        err = clEnqueueWriteBuffer(queue, obstacleBuffer, CL_TRUE, 0, sizeof(Obstacle) * NB_OBSTACLES, obstacles, 0, NULL, NULL);
        printf("clEnqueueWriteBuffer: %d\n", err);
        if (err != CL_SUCCESS) return;
        
        // Prepare constants
        float gravity = GRAVITY;
        float maxLeft = MAX_LEFT;
        float maxRight = MAX_RIGHT;
        float maxTop = MAX_TOP;
        float maxBottom = MAX_BOTTOM;
        float damping = DAMPING_FACTOR;
        int nbObstacles = NB_OBSTACLES;
        int gridWidth = GRID_WIDTH;
        int gridHeight = GRID_HEIGHT;

        // Set kernel 1 arguments
        err = clSetKernelArg(kernel1, 0, sizeof(cl_mem), &particleBuffer);
        err |= clSetKernelArg(kernel1, 1, sizeof(cl_mem), &obstacleBuffer);
        err |= clSetKernelArg(kernel1, 2, sizeof(float), &gravity);
        err |= clSetKernelArg(kernel1, 3, sizeof(float), &maxLeft);
        err |= clSetKernelArg(kernel1, 4, sizeof(float), &maxRight);
        err |= clSetKernelArg(kernel1, 5, sizeof(float), &maxTop);
        err |= clSetKernelArg(kernel1, 6, sizeof(float), &maxBottom);
        err |= clSetKernelArg(kernel1, 7, sizeof(float), &damping);
        err |= clSetKernelArg(kernel1, 8, sizeof(float), &nbObstacles);
        if (err != CL_SUCCESS) {
			printf("Failed to set OpenCL kernel args: %d\n", err);
			return;
		}

        // Set kernel 2 arguments
        err = clSetKernelArg(kernel2, 0, sizeof(cl_mem), &particleBuffer);
        err |= clSetKernelArg(kernel2, 1, sizeof(cl_mem), &gridBuffer);
        err |= clSetKernelArg(kernel2, 2, sizeof(int), &gridWidth);
        err |= clSetKernelArg(kernel2, 3, sizeof(int), &gridHeight);
        err |= clSetKernelArg(kernel2, 4, sizeof(float), &damping);
        if (err != CL_SUCCESS) {
            printf("Failed to set OpenCL kernel args: %d\n", err);
            return;
        }

        printf("OpenCL initialization completed successfully.\n");
	}
}

// Sequential version
void UpdateSimulation_Sequential() {
    for (int i = 0; i < NB_PARTICLES; i++) {
        particles[i].velocity.y += GRAVITY;
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;
        ResolveBoundaryCollisions(&particles[i]);
        ResolveObstacleCollisions(&particles[i], obstacles);
    }
    ResolveParticleCollisions(particles);
}

// CPU threading version
void AssignParticlesToGrid() {
    // Step 1: Reset the grid cell counts to zero
    for (int x = 0; x < GRID_WIDTH; x++) {
        for (int y = 0; y < GRID_HEIGHT; y++) {
            grid[x][y].count = 0;  // Reset count of particles in each cell
        }
    }

    // Step 2: Assign particles to their respective grid cells
    for (int i = 0; i < NB_PARTICLES; i++) {
        int x = (int)(particles[i].position.x / GRID_CELL_WIDTH);
        int y = (int)(particles[i].position.y / GRID_CELL_HEIGHT);

        // Ensure x and y are within the grid's bounds
        if (x >= GRID_WIDTH) x = GRID_WIDTH - 1;
        if (y >= GRID_HEIGHT) y = GRID_HEIGHT - 1;

        int count = grid[x][y].count;
        grid[x][y].particles[count] = &particles[i];  // Append particle
        grid[x][y].count++;  // Increase count
    }
}

void* ProcessBoundariesAndObstacles(void* arg) {
    CpuThreadData* data = (CpuThreadData*)arg;

    for (int i = data->threadIndex; i < NB_PARTICLES; i += NUM_THREADS_CPU) {
        particles[i].velocity.y += GRAVITY;
        particles[i].position.x += particles[i].velocity.x;
        particles[i].position.y += particles[i].velocity.y;
        ResolveBoundaryCollisions(&particles[i]);
        ResolveObstacleCollisions(&particles[i], obstacles);
    }

    return NULL;
}

void* ProcessParticleCollisions(void* arg) {
    CpuThreadData* data = (CpuThreadData*)arg;

    for (int cellIndex = data->threadIndex; cellIndex < GRID_WIDTH * GRID_HEIGHT; cellIndex += NUM_THREADS_CPU) {
        int x = cellIndex % GRID_WIDTH;
        int y = cellIndex / GRID_WIDTH;

        // Skip empty cells (no particles)
        if (grid[x][y].count == 0) continue;

        GridCell* cell = &grid[x][y];

        // Check collisions within the cell
        for (int i = 0; i < cell->count; i++) {
            for (int j = i + 1; j < cell->count; j++) {
                ResolveParticleCollision(cell->particles[i], cell->particles[j]);
            }
        }

        // Check neighboring cells (right & bottom) only for particles near borders
        if (x + 1 < GRID_WIDTH && grid[x + 1][y].count > 0) {
            GridCell* right = &grid[x + 1][y];
            for (int i = 0; i < cell->count; i++) {
                if (cell->particles[i]->position.x + cell->particles[i]->radius > (x + 1) * GRID_CELL_WIDTH) {
                    for (int j = 0; j < right->count; j++) {
                        ResolveParticleCollision(cell->particles[i], right->particles[j]);
                    }
                }
            }
        }

        if (y + 1 < GRID_HEIGHT && grid[x][y + 1].count > 0) {
            GridCell* below = &grid[x][y + 1];
            for (int i = 0; i < cell->count; i++) {
                if (cell->particles[i]->position.y + cell->particles[i]->radius > (y + 1) * GRID_CELL_HEIGHT) {
                    for (int j = 0; j < below->count; j++) {
                        ResolveParticleCollision(cell->particles[i], below->particles[j]);
                    }
                }
            }
        }
    }

    return NULL;
}

void UpdateSimulation_CpuThreading() {
    // Step 1: Assign particles to the grid
    AssignParticlesToGrid();

    // Step 2: Handle boundary & obstacle collisions
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        threadData[i].threadIndex = i;
        pthread_create(&threads[i], NULL, ProcessBoundariesAndObstacles, &threadData[i]);
    }
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_join(threads[i], NULL);
    }

    // Step 3: Handle particle collisions per grid cell
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        threadData[i].threadIndex = i;
        pthread_create(&threads[i], NULL, ProcessParticleCollisions, &threadData[i]);
    }
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_join(threads[i], NULL);
    }
}

void UpdateSimulationOpenCL() {
    // Launch the kernel to update particles (boundary + obstace collisions)
    size_t globalSize = NB_PARTICLES;
    err = clEnqueueNDRangeKernel(queue, kernel1, 1, NULL, &globalSize, NULL, 0, NULL, NULL);
    clFinish(queue);

    // Read results from the GPU
    clEnqueueReadBuffer(queue, particleBuffer, CL_TRUE, 0, sizeof(Particle) * NB_PARTICLES, particles, 0, NULL, NULL);

    // Handle particle collisions
    // Assign particles to grid cells
    size_t gridSize = GRID_WIDTH * GRID_HEIGHT;
    GridCellCL* grid = (GridCellCL*)calloc(gridSize, sizeof(GridCellCL));
    if (grid == NULL) {
        // Print error and exit if memory allocation fails
        fprintf(stderr, "Failed to allocate memory for the grid.\n");
        exit(1);
    }

    for (int i = 0; i < NB_PARTICLES; i++) {
        int gx = particles[i].position.x / GRID_CELL_WIDTH;
        int gy = particles[i].position.y / GRID_CELL_HEIGHT;
        int cellIndex = gy * GRID_WIDTH + gx;

        GridCellCL* cell = &grid[cellIndex];
        if (cell->count < MAX_PARTICLES_PER_CELL) {
            cell->indices[cell->count++] = i;
        }
    }

    // Write the grid to the OpenCL buffer
    err = clEnqueueWriteBuffer(queue, gridBuffer, CL_TRUE, 0, sizeof(GridCellCL) * gridSize, grid, 0, NULL, NULL);

    free(grid);

    // Launch the kernel to handle particle collisions
    err = clEnqueueNDRangeKernel(queue, kernel2, 1, NULL, &gridSize, NULL, 0, NULL, NULL);
    clFinish(queue);

    // Read results from the GPU
    clEnqueueReadBuffer(queue, particleBuffer, CL_TRUE, 0, sizeof(Particle) * NB_PARTICLES, particles, 0, NULL, NULL);
}

void UpdateSimulation() {
    switch (executionMode)
    {
    case EXECUTION_CPU_THREADING:
        UpdateSimulation_CpuThreading();
        break;
    case EXECUTION_OPENCL_GPU:
        UpdateSimulationOpenCL();
		break;
    case EXECUTION_SEQUENTIAL:
        UpdateSimulation_Sequential();
        break;
    default:
        UpdateSimulation_Sequential();
        break;
    }
}

Particle* GetParticles() {
    return particles;
}

Obstacle* GetObstacles() {
    return obstacles;
}

void CleanupSimulation() {
    if (executionMode == EXECUTION_OPENCL_GPU) {
		clReleaseMemObject(particleBuffer);
		clReleaseKernel(kernel1);
        clReleaseKernel(kernel2);
		clReleaseProgram(program);
		clReleaseCommandQueue(queue);
		clReleaseContext(context);
	}
}