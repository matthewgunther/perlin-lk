#ifndef FLOW_FIELD_H
#define FLOW_FIELD_H

#include <iostream>
#include "particle.h"
#include "PerlinNoise.hpp"


#define FLOW_SCALE 100
#define PADDING 130
#define TIMESTEP 0.2

#define PERLIN_ARR_SCALE 2
#define X_SCALAR 0.1
#define Y_SCALAR 0.1
#define Z_DELTA 0.01

#define VEL_DAMPEN_COEFF 0.125
#define ACC_DAMPEN_COEFF 0.25


using namespace std;
using namespace siv;

class FlowField {
    public:
        int num_of_particles;


    private:
        const PerlinNoise::seed_type seed = 123456u;
        const PerlinNoise perlin { seed };
        float perlin_z { 0 };
    

    public:
        void initialize_particles (Particle particles[], int num_of_particles);
        void move_particles (
            Particle particles[], 
            int num_of_particles,
            int rows, 
            int cols, 
            float downsample_scale
        );
};

#endif