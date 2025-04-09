#if defined(__cplusplus)
extern "C"
{
#endif
    #include <simulation.h>

    void UpdateSimulationCuda(Particle* particles, GridCellGPU* grid, Particle* d_particles, Obstacle* d_obstacles, GridCellGPU* d_grid);
#if defined(__cplusplus)
}
#endif