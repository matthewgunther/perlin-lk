#include "particle.h"
#include <iostream>

using namespace std;

float check_window_bound(float value, float bound) {
    if (value < 0) {
        value += bound;
    } else if (value > bound) {
        value -= bound;
    }
    return value;
}

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

void Particle::update(float timestep, int window_width, int window_height) {
    pos.update(vel, timestep);
    vel.update(acc, timestep);
    pos.x = check_window_bound(pos.x, (float)window_width);
    pos.y = check_window_bound(pos.y, (float)window_height);
    vel.check_magnitude_limit();
}