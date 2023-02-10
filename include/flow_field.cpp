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


void FlowField::test (Particle arr[]) {
    // cout << perlin.octave3D_01((1 * 0.1), (1 * 0.1), 0, 4) << endl;
    arr[1].pos.x = 16;
}

// // double** FlowField::construct_perlin_array (
// //     int rows,
// //     int cols,
// //     float x_scalar, 
// //     float y_scalar, 
// //     float z_delta
// // ) {

// //     double** noise_arr;
// //     for (int y = 0; y < rows; y++) {
// //         for (int x = 0; x < cols; x++) {
// //             noise_arr[y][x] = perlin.octave3D_01((x * x_scalar), (y * y_scalar), perlin_z, 4);
// //         }
// //     }
// //     perlin_z += z_delta;

// //     return noise_arr;

// // }



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
            float timestep { 0.6 };

            int arr_x = floor(particles[i].pos.x / downsample_scale / num_of_particles);
            int arr_y = floor(particles[i].pos.y / downsample_scale / num_of_particles);


            float noise_angle = noise_arr[arr_y][arr_x];

            float fl_scale { 10 };
            float fl_x = cos(noise_angle);
            float fl_y = sin(noise_angle);

            particles[i].acc.add(fl_x * fl_scale, fl_y * fl_scale); 

    

            // float add_x = u.at<float>(int(bubbles[i].pos.y / scale), int(bubbles[i].pos.x / scale));
            // float add_y = v.at<float>(int(bubbles[i].pos.y / scale), int(bubbles[i].pos.x / scale));


            
            // if (sqrtf32(pow(add_x, 2) + pow(add_y, 2)) > flow_threshold) {

            //     float acc_scale { 5 };

            //     float angle = atanf32(add_y / add_x) * 180 / M_PI;
            //     angle = map_atan_to_360_deg(add_x, add_y, angle); // maps arctan output to 360 degrees
            //     Vec3b color = get_rgb_from_hsv(angle);
            //     bubbles[i].acc.add(add_x * acc_scale, add_y * acc_scale); 
            //     bubbles[i].update(timestep, current_frame_color.cols, current_frame_color.rows, padding);
            //     circle(current_frame_color, Point(int(bubbles[i].pos.x), int(bubbles[i].pos.y)), 3, color, 2);

            // } else {
            //     bubbles[i].update(timestep, current_frame_color.cols, current_frame_color.rows, padding);
            //     circle(current_frame_color, Point(int(bubbles[i].pos.x), int(bubbles[i].pos.y)), 3, Scalar(255, 255, 255), 2);
            // }

            particles[i].update(
                timestep, 
                cols, 
                rows, 
                padding
            );

            particles[i].acc.x = 0;
            particles[i].acc.y = 0;
            particles[i].vel.dampening();
            // bubbles[i].acc.dampening();
        }

}



// void FlowField::initialize_perlin_array (int rows, int cols) {
//     arr
// }