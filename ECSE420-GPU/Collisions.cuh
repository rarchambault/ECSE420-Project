#ifndef COLLISIONS_H
#define COLLISIONS_H

#include "constants.cuh"
#include <cuda_runtime.h>


#ifdef __cplusplus
extern "C" {
#endif

    typedef struct {
        float2 position;
        float2 velocity;
        float  radius;
    } Particle;


    typedef struct {
        float2 position;
        float radius;
    } Obstacle;




#ifdef __cplusplus
}
#endif

#endif // COLLISIONS_H
