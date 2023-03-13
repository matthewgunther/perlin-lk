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
    uniform_int_distribution<> distr_pos_x(PADDING, cols-PADDING);
    uniform_int_distribution<> distr_pos_y(PADDING, rows-PADDING); 
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

void FlowField::move_particles (
    Particle particles[],
    int num_of_particles,
    int* rows, 
    int* cols
    ) {
        
    int x_lim = floor(*rows / NOISE_ARR_SCALE);
    int y_lim = floor(*cols / NOISE_ARR_SCALE);
    
    // compute perlin flow field
    double noise_arr[x_lim][y_lim];
    for (int y = 0; y < x_lim; y++) {
        for (int x = 0; x < y_lim; x++) {
            noise_arr[y][x] = perlin.octave3D_01((x * NOISE_X_SCALAR), (y * NOISE_Y_SCALAR), perlin_z, 4) * M_PI * 4;
        }
    }
    perlin_z += NOISE_Z_DELTA;


    for (int i = 0; i < num_of_particles; i++) {
        
        // get flow vector from flow field
        int arr_x = floor(particles[i].pos.x / DOWNSAMPLE_SCALE / num_of_particles);
        int arr_y = floor(particles[i].pos.y / DOWNSAMPLE_SCALE / num_of_particles);
        float noise_angle = noise_arr[arr_y][arr_x];
        float flow_x = cos(noise_angle);
        float flow_y = sin(noise_angle);

        // add optical flow acceleration, user "pushing" particles
        add(&particles[i].acc, (flow_x * FLOW_SCALE), (flow_y * FLOW_SCALE));
        // update physics
        update_vec(&particles[i].vel, &particles[i].acc);
        update_vec(&particles[i].pos, &particles[i].vel);
        // keep particles in window
        check_window_bound(&particles[i].pos, (float)*cols, (float)*rows);
        // keep particles within speed range
        check_magnitude_limit(&particles[i].vel);
        check_magnitude_limit(&particles[i].acc);
        // dampen the motion of particles after push
        dampen(&particles[i].vel, VEL_DAMPEN_COEFF);
        dampen(&particles[i].acc, ACC_DAMPEN_COEFF);
    }
}