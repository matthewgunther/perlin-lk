#ifndef FLOW_FIELD_H
#define FLOW_FIELD_H

#include <iostream>
#include "particle.h"
#include "PerlinNoise.hpp"




using namespace std;
using namespace siv;

class FlowField {
    public:
        int num_of_particles;

    // private:
        const PerlinNoise::seed_type seed = 123456u;
        const PerlinNoise perlin { seed };
        float perlin_z { 0 };

    public:
        void initialize_particles (
            Particle particles[], 
            unordered_map<int, vector<int>>& particle_hash,
            int rows,
            int cols
        );
        void move_particles (
            Particle particles[], 
            int num_of_particles,
            int* rows, 
            int* cols
        );
};

#endif