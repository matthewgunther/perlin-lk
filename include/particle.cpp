#include "particle.h"
#include <iostream>

using namespace std;

// helper functions
float check_window_bound(float value, float bound, float padding) {
    if (value < padding) {
        value += (bound - padding * 2);
    } else if (value > (bound - padding)) {
        value -= (bound - padding * 2);
    }
    return value;
}


// vector method definitions
void Particle::vec::add (float x_to_add, float y_to_add) {
    if (isnanf(x_to_add) || abs(x_to_add) <= 1 || abs(x_to_add) > 100)
        x_to_add = 0;
    if (isnanf(y_to_add) || abs(y_to_add) <= 1 || abs(y_to_add) > 100)
        y_to_add = 0;

    x += x_to_add;
    y += y_to_add;
    check_magnitude_limit();
}

void Particle::vec::dampening () {
    x -= x * dampening_coeff;
    y -= y * dampening_coeff;
}

void Particle::vec::update (vec vector, float timestep) {
    x += vector.x * timestep;
    y += vector.y * timestep;
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

void Particle::update(float timestep, int window_width, int window_height, int padding) {
    pos.update(vel, timestep);
    vel.update(acc, timestep);
    pos.x = check_window_bound(pos.x, (float)window_width, (float)padding);
    pos.y = check_window_bound(pos.y, (float)window_height, (float)padding);
    vel.check_magnitude_limit();
}