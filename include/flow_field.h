#ifndef FLOW_FIELD_H
#define FLOW_FIELD_H

#include <iostream>
#include "particle.h"
#include "PerlinNoise.hpp"


#define FLOW_SCALE 10
#define PADDING 130
#define TIMESTEP 0.2


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
        void test (Particle arr[]);
        void move_particles (
            Particle particles[], 
            int num_of_particles,
            int rows, 
            int cols, 
            int perlin_scale,
            float x_scalar, 
            float y_scalar, 
            float z_delta,
            float downsample_scale
        );
};

#endif