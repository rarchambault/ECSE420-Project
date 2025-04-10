#include "simulation.h"
#include "constants.h"
#include "particle.h"
#include "threading.h"
#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <CL/cl.h>
#include <cuda_runtime.h>

GridCell grid[GRID_WIDTH][GRID_HEIGHT]; // The simulation grid for CPU threading
GridCellGPU gridGPU[GRID_WIDTH][GRID_HEIGHT]; // The simulation grid for OpenCL and CUDA
Particle particles[NB_PARTICLES];
Obstacle obstacles[NB_OBSTACLES] = {
    {100, 150, 30.0f},
    {400, 300, 50.0f},
    {200, 400, 20.0f},
    {600, 200, 40.0f},
    {700, 500, 60.0f},
    {800, 100, 10.0f}
};

pthread_t threads[NUM_THREADS_CPU];

CpuThreadData threadData[NUM_THREADS_CPU];

cl_platform_id platform;
cl_device_id device;
cl_context context;
cl_command_queue queue;
cl_program program;
cl_kernel kernel1;
cl_kernel kernel2;
cl_kernel kernel3;
cl_kernel kernel4;
cl_int err;
cl_mem particleBuffer;
cl_mem gridBuffer;
cl_mem obstacleBuffer;

Particle* d_particles;
Obstacle* d_obstacles;
GridCellGPU* d_grid;

void InitSimulation() {
    srand(RANDOM_SEED);
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

    if (executionMode == EXECUTION_GPU_OPENCL || executionMode == EXECUTION_CPU_GRAPHICS_OPENCL || executionMode == EXECUTION_CPU_OPENCL) {
        cl_uint numPlatforms;
        cl_platform_id platforms[5];
        err = clGetPlatformIDs(5, platforms, &numPlatforms);
        if (err != CL_SUCCESS || numPlatforms == 0) {
            fprintf(stderr, "clGetPlatformIDs failed: %d\n", err);
            exit(1);
        }

        for (cl_uint i = 0; i < numPlatforms; i++) {
            // Get name of platform
            char platformName[128]= { 0 };
            clGetPlatformInfo(platforms[i], CL_PLATFORM_NAME, sizeof(platformName), platformName, NULL);
            
            // NVIDIA CPU
            if (executionMode == EXECUTION_GPU_OPENCL) {
                if (strcmp(platformName, "NVIDIA CUDA") == 0) {
					platform = platforms[i];
					break;
				}
            }
            // Intel(R) OpenCL HD Graphics
            if (executionMode == EXECUTION_CPU_GRAPHICS_OPENCL) {
				if (strcmp(platformName, "Intel(R) OpenCL HD Graphics") == 0) {
                    platform = platforms[i];
					break;
				}
             }
			// Intel(R) OpenCL CPU
			if (executionMode == EXECUTION_CPU_OPENCL) {
                if (strcmp(platformName, "Intel(R) OpenCL") == 0) {
					platform = platforms[i];
					break;
				}
			}
		}

		if (!platform) {
			fprintf(stderr, "No suitable OpenCL platform found.\n");
			exit(1);
		}

        // Set OpenCL device
        cl_device_type deviceType = CL_DEVICE_TYPE_GPU;
        if (executionMode == EXECUTION_CPU_OPENCL) deviceType = CL_DEVICE_TYPE_CPU;
		err = clGetDeviceIDs(platform, deviceType, 1, &device, NULL);
		if (err != CL_SUCCESS) {
			fprintf(stderr, "clGetDeviceIDs failed: %d\n", err);
			exit(1);
		}

        // Print device name
        char deviceName[128];
        clGetDeviceInfo(device, CL_DEVICE_NAME, sizeof(deviceName), deviceName, NULL);
        printf("Using OpenCL device: %s\n", deviceName);

        // Create OpenCL context and command queue
        context = clCreateContext(NULL, 1, &device, NULL, NULL, &err);
        if (err != CL_SUCCESS) {
			fprintf(stderr, "clCreateContext failed: %d\n", err);
			exit(1);
		}

        queue = clCreateCommandQueue(context, device, 0, &err);
        if (err != CL_SUCCESS) {
            fprintf(stderr, "clCreateCommandQueue failed: %d\n", err);
            exit(1);
        }

        // Load and compile OpenCL kernel
        FILE* file = fopen("src/particle.cl", "rb");
        fseek(file, 0, SEEK_END);
        size_t sourceSize = ftell(file);
        rewind(file);
        char* sourceCode = (char*)malloc(sourceSize + 1);
        if (!sourceCode) {
            fprintf(stderr, "Failed to allocate memory for kernel source\n");
            fclose(file);
            exit(1);
        }
        size_t bytesRead = fread(sourceCode, 1, sourceSize, file);
        fclose(file);
        size_t sourceSizes[1] = { sourceSize };
        program = clCreateProgramWithSource(context, 1, &sourceCode, sourceSizes, &err);
        
        if (err != CL_SUCCESS) {
            fprintf(stderr, "clCreateProgramWithSource failed with error code %d\n", err);
			free(sourceCode);
			return;
		}

        free(sourceCode);

        // Build the OpenCL program
        err = clBuildProgram(program, 1, &device, NULL, NULL, NULL);
        if (err != CL_SUCCESS) {
			size_t logSize;
			clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, 0, NULL, &logSize);
			char* log = (char*)malloc(logSize);
			clGetProgramBuildInfo(program, device, CL_PROGRAM_BUILD_LOG, logSize, log, NULL);
			fprintf(stderr, "clBuildProgram failed: %d\n", err);
			fprintf(stderr, "Build log:\n%s\n", log);
			free(log);
            exit(1);
		}

        // Create OpenCL kernels
        kernel1 = clCreateKernel(program, "UpdateParticles", &err);
        kernel2 = clCreateKernel(program, "ProcessGridCollisions", &err);
        kernel3 = clCreateKernel(program, "AssignParticlesToGrid", &err);
        kernel4 = clCreateKernel(program, "ResetGrid", &err);

        if (err != CL_SUCCESS) {
            fprintf(stderr, "clCreateKernel failed: %d\n", err);
            exit(1);
        }

        // Create OpenCL buffers
        particleBuffer = clCreateBuffer(context, CL_MEM_READ_WRITE, sizeof(Particle) * NB_PARTICLES, NULL, &err);
        gridBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(GridCellGPU) * (GRID_WIDTH * GRID_HEIGHT), NULL, &err);
        obstacleBuffer = clCreateBuffer(context, CL_MEM_READ_ONLY, sizeof(Obstacle) * NB_OBSTACLES, NULL, &err);
        if (err != CL_SUCCESS) {
			fprintf(stderr, "clCreateBuffer failed: %d\n", err);
			exit(1);
		}

        // Copy initial data to OpenCL buffers
        err = clEnqueueWriteBuffer(queue, particleBuffer, CL_TRUE, 0, sizeof(Particle) * NB_PARTICLES, particles, 0, NULL, NULL);
        err |= clEnqueueWriteBuffer(queue, gridBuffer, CL_TRUE, 0, sizeof(GridCellGPU) * (GRID_WIDTH * GRID_HEIGHT), gridGPU, 0, NULL, NULL);
        err |= clEnqueueWriteBuffer(queue, obstacleBuffer, CL_TRUE, 0, sizeof(Obstacle) * NB_OBSTACLES, obstacles, 0, NULL, NULL);
        if (err != CL_SUCCESS) {
            fprintf(stderr, "clEnqueueWriteBuffer failed: %d\n", err);
            exit(1);
        }
        
        // Prepare constants
        float gravity = GRAVITY;
        float maxLeft = MAX_LEFT;
        float maxRight = MAX_RIGHT;
        float maxTop = MAX_TOP;
        float maxBottom = MAX_BOTTOM;
        float damping = DAMPING_FACTOR;
        int nbObstacles = NB_OBSTACLES;
        int n = NB_PARTICLES;
        int gridWidth = GRID_WIDTH;
        int gridHeight = GRID_HEIGHT;
        float gridCellWidth = GRID_CELL_WIDTH;
        float gridCellHeight = GRID_CELL_HEIGHT;
        int maxParticlesPerCell = MAX_PARTICLES_PER_CELL;

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

        // Set kernel 2 arguments
        err |= clSetKernelArg(kernel2, 0, sizeof(cl_mem), &particleBuffer);
        err |= clSetKernelArg(kernel2, 1, sizeof(cl_mem), &gridBuffer);
        err |= clSetKernelArg(kernel2, 2, sizeof(int), &gridWidth);
        err |= clSetKernelArg(kernel2, 3, sizeof(int), &gridHeight);
        err |= clSetKernelArg(kernel2, 4, sizeof(float), &gridCellWidth);
        err |= clSetKernelArg(kernel2, 5, sizeof(float), &gridCellHeight);
        err |= clSetKernelArg(kernel2, 6, sizeof(float), &damping);

        // Set kernel 3 arguments
        err |= clSetKernelArg(kernel3, 0, sizeof(cl_mem), &particleBuffer);
        err |= clSetKernelArg(kernel3, 1, sizeof(cl_mem), &gridBuffer);
        err |= clSetKernelArg(kernel3, 2, sizeof(int), &n);
        err |= clSetKernelArg(kernel3, 3, sizeof(int), &gridWidth);
        err |= clSetKernelArg(kernel3, 4, sizeof(int), &gridHeight);
        err |= clSetKernelArg(kernel3, 5, sizeof(float), &gridCellWidth);
        err |= clSetKernelArg(kernel3, 6, sizeof(float), &gridCellHeight);
        err |= clSetKernelArg(kernel3, 7, sizeof(int), &maxParticlesPerCell);

        // Set kernel 4 arguments
        err |= clSetKernelArg(kernel4, 0, sizeof(cl_mem), &gridBuffer);
        err |= clSetKernelArg(kernel4, 1, sizeof(int), &gridWidth);
        err |= clSetKernelArg(kernel4, 2, sizeof(int), &gridHeight);

        if (err != CL_SUCCESS) {
            fprintf(stderr, "Failed to set OpenCL kernel args: %d\n", err);
            exit(1);
        }

        printf("OpenCL initialization completed successfully.\n");
        
	} else if (executionMode == EXECUTION_GPU_CUDA) {
        cudaError_t err;

        err = cudaMalloc(&d_particles, sizeof(Particle) * NB_PARTICLES);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMalloc failed for particles: %s\n", cudaGetErrorString(err));
            exit(1);
        }

        // 1. Allocate device memory for obstacles
        err = cudaMalloc(&d_obstacles, sizeof(Obstacle) * NB_OBSTACLES);
        if (err != cudaSuccess) {
			fprintf(stderr, "cudaMalloc failed for obstacles: %s\n", cudaGetErrorString(err));
			exit(1);
		}

        // 2. Allocate device memory for grid
        size_t gridSize = GRID_WIDTH * GRID_HEIGHT;
        err = cudaMalloc(&d_grid, sizeof(GridCellGPU) * gridSize);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMalloc failed for grid: %s\n", cudaGetErrorString(err));
            exit(1);
        }

        // 3. Copy initial particle data to device
        err = cudaMemcpy(d_particles, particles, sizeof(Particle) * NB_PARTICLES, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
            fprintf(stderr, "cudaMemcpy failed for particles: %s\n", cudaGetErrorString(err));
            exit(1);
        }

        // 4. Copy initial obstacle data to device
        err = cudaMemcpy(d_obstacles, obstacles, sizeof(Obstacle) * NB_OBSTACLES, cudaMemcpyHostToDevice);
        if (err != cudaSuccess) {
			fprintf(stderr, "cudaMemcpy failed for obstacles: %s\n", cudaGetErrorString(err));
			exit(1);
		}

        printf("CUDA GPU memory initialized.\n");
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

void UpdateSimulation_CpuThreading() {
    // Step 1: Handle boundary & obstacle collisions
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        threadData[i].threadIndex = i;
        pthread_create(&threads[i], NULL, ProcessBoundariesAndObstacles, &threadData[i]);
    }
    for (int i = 0; i < NUM_THREADS_CPU; i++) {
        pthread_join(threads[i], NULL);
    }

    // Step 2: Assign particles to the grid
    AssignParticlesToGrid();

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
    err |= clEnqueueNDRangeKernel(queue, kernel1, 1, NULL, &globalSize, NULL, 0, NULL, NULL);

    // Reset the grid
    size_t gridSize = GRID_WIDTH * GRID_HEIGHT;
    err |= clEnqueueNDRangeKernel(queue, kernel4, 1, NULL, &gridSize, NULL, 0, NULL, NULL);

    // Wait to ensure the kernel execution is finished
    clFinish(queue);

    // Assign particles to the grid
    err |= clEnqueueNDRangeKernel(queue, kernel3, 1, NULL, &globalSize, NULL, 0, NULL, NULL);

    // Wait to ensure the kernel execution is finished
    clFinish(queue);

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
    case EXECUTION_GPU_OPENCL:
        UpdateSimulationOpenCL();
		break;
    case EXECUTION_CPU_GRAPHICS_OPENCL:
		UpdateSimulationOpenCL();
        break;
    case EXECUTION_CPU_OPENCL:
        UpdateSimulationOpenCL();
		break;
    case EXECUTION_GPU_CUDA:
        UpdateSimulationCuda(particles, gridGPU, d_particles, d_obstacles, d_grid);
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
    if (executionMode == EXECUTION_GPU_OPENCL) {
		clReleaseMemObject(particleBuffer);
        clReleaseMemObject(gridBuffer);
        clReleaseMemObject(obstacleBuffer);
		clReleaseKernel(kernel1);
        clReleaseKernel(kernel2);
        clReleaseKernel(kernel3);
        clReleaseKernel(kernel4);
		clReleaseProgram(program);
		clReleaseCommandQueue(queue);
		clReleaseContext(context);
        clReleaseDevice(device);
	} else if (executionMode == EXECUTION_GPU_CUDA) {
		cudaFree(d_particles);
		cudaFree(d_grid);
        cudaFree(d_obstacles);
	}
}