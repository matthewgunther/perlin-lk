#include "engine.h"
#include "flow_field.h"
#include "particle.h"

#include "PerlinNoise.hpp"

#define _USE_MATH_DEFINES

#include <cmath>
#include <iostream>
#include <random>

using namespace std;


void FlowField::initialize_particles (
    Particle particles[], 
    unordered_map<int, vector<int>>& particle_hash,
    int rows,
    int cols
    ) {

    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr_pos_x(PADDING, cols - PADDING);
    uniform_int_distribution<> distr_pos_y(PADDING, rows - PADDING); 
    uniform_int_distribution<> distr_vec(-10, 10); // define the range for velocities

    // assign initial values
    for (int i = 0; i < NUM_OF_BUBBLES; i++) {

        // white circle
        Vec3b color;
        color[0] = 255;
        color[1] = 255;
        color[2] = 255;
        particles[i].color = color;

        particles[i].pos.x = distr_pos_x(gen);
        particles[i].pos.y = distr_pos_y(gen);
        particles[i].vel.x = 0;
        particles[i].vel.y = 0;
        particles[i].acc.x = 0;
        particles[i].acc.y = 0;

        particles[i].vel.magnitude_limit = 100;
        particles[i].acc.magnitude_limit = 1000;

        particles[i].vel.dampening_coeff = 0.125;
        particles[i].acc.dampening_coeff = 0.25;

        // linear index for each point
        int key = floor(particles[i].pos.y / DOWNSAMPLE_SCALE)
            * floor(cols / DOWNSAMPLE_SCALE) 
            + floor(particles[i].pos.x / DOWNSAMPLE_SCALE);

        if (particle_hash.find(key) == particle_hash.end()) {
            // not found
            particle_hash[key] = {i};
        } else {
            particle_hash[key].push_back(i);
        }
    }
}