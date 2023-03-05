#include "flow_field.h"
#include "particle.h"
#include "PerlinNoise.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

#include <iostream>
#include <random>

using namespace std;


void FlowField::initialize_particles (Particle particles[], int num_of_particles) {

    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr_pos(100, 900); // define the range
    uniform_int_distribution<> distr_vec(-10, 10); // define the range


    // length of array = size of particles array divided by size of one particle object

    for (int i = 0; i < num_of_particles; i++) {

        particles[i].initialize_vectors(
            distr_pos(gen), 
            distr_pos(gen), 
            0, 0, 
            // distr_vec(gen), distr_vec(gen),
            0, 
            0
        );

        particles[i].vel.magnitude_limit = 60;
        particles[i].acc.magnitude_limit = 100;

        particles[i].vel.dampening_coeff = 0.125;
        particles[i].acc.dampening_coeff = 0.25;
    }
}


void FlowField::move_particles (
    Particle particles[],
    int num_of_particles,
    int rows, 
    int cols, 
    int perlin_scale,
    float x_scalar, 
    float y_scalar, 
    float z_delta,
    float downsample_scale
    ) {
        
        int x_lim = floor(rows / perlin_scale);
        int y_lim = floor(cols / perlin_scale);

        double noise_arr[x_lim][y_lim];
        for (int y = 0; y < x_lim; y++) {
            for (int x = 0; x < y_lim; x++) {
                noise_arr[y][x] = perlin.octave3D_01((x * x_scalar), (y * y_scalar), perlin_z, 4) * M_PI * 4;
            }
        }
        perlin_z += z_delta;



        for (int i = 0; i < num_of_particles; i++) {
            int padding {130};

            int arr_x = floor(particles[i].pos.x / downsample_scale / num_of_particles);
            int arr_y = floor(particles[i].pos.y / downsample_scale / num_of_particles);


            float noise_angle = noise_arr[arr_y][arr_x];

            float flow_x = cos(noise_angle);
            float flow_y = sin(noise_angle);

            add(&particles[i].acc, (flow_x * FLOW_SCALE), (flow_y * FLOW_SCALE));

            particles[i].update(
                cols, 
                rows
            );

            dampen(&particles[i].vel);
            dampen(&particles[i].acc);
        }

}
