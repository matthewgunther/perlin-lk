#pragma once

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;


class Particle {
    public:
        struct vec {
            float x, y; // keep
            float magnitude_limit { 3.0e+038 };
            float dampening_coeff { 0 };
        } pos, vel, acc;
        Vec3b color;
        Particle() {}  
};

void add(Particle::vec* vec_pointer, float x_to_add, float y_to_add);
void check_magnitude_limit (Particle::vec* vec_pointer);
void check_window_bound (Particle::vec* vec_pointer, float bound_x, float bound_y);
void dampen (Particle::vec* vec_pointer, float dampen_coeff);
void update_vec (Particle::vec* vec_one_pointer, Particle::vec* vec_two_pointer);
