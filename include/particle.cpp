#include "particle.h"
#include <iostream>

using namespace std;


// helper functions
float check_window_bound(float value, float bound) {
    if (value < PADDING) {
        value += (bound - PADDING * 2);
    } else if (value > (bound - PADDING)) {
        value -= (bound - PADDING * 2);
    }
    return value;
}


// vector method definitions
void add (Particle::vec* vec_var, float x_to_add, float y_to_add) {

    if (isnanf(x_to_add) || abs(x_to_add) <= 1) {
        x_to_add = 0;
    }
    if (isnanf(y_to_add) || abs(y_to_add) <= 1) {
        y_to_add = 0;
    }
    vec_var->x += x_to_add;
    vec_var->y += y_to_add;
    vec_var->check_magnitude_limit();
}

void Particle::vec::dampening () {
    x -= x * dampening_coeff;
    y -= y * dampening_coeff;
}

void dampen(Particle::vec* vec_var) {
    vec_var->x = vec_var->x * DAMPENING_COEFF;
    vec_var->y = vec_var->y * DAMPENING_COEFF;
}

void Particle::vec::check_magnitude_limit () {
    if (fabs(x) > magnitude_limit) {
        if (x > magnitude_limit) {
            x = magnitude_limit;
        } else {
            x = -1 * magnitude_limit;
        }
    }
    if (fabs(y) > magnitude_limit) {
        if (y > magnitude_limit) {
            y = magnitude_limit;
        } else {
            y = -1 * magnitude_limit;
        }
    }
}


// Particle class methods
void Particle::initialize_vectors(
        float pos_x, float pos_y,
        float vel_x, float vel_y,
        float acc_x, float acc_y
    ) {
    pos.x = pos_x;
    pos.y = pos_y;
    vel.x = vel_x;
    vel.y = vel_y;
    acc.x = acc_x;
    acc.y = acc_y;
}

void update_vec(Particle::vec* vec_one, Particle::vec* vec_two){
    vec_one->x += vec_two->x * TIMESTEP;
    vec_one->y += vec_two->y * TIMESTEP;
}

void Particle::update(int window_width, int window_height) {
    update_vec(&pos, &vel);
    update_vec(&vel, &acc);
    pos.x = check_window_bound(pos.x, (float)window_width);
    pos.y = check_window_bound(pos.y, (float)window_height);
    vel.check_magnitude_limit();
}