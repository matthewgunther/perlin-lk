#include "particle.h"
#include <iostream>

using namespace std;

void Particle::set_pos(float x, float y) {
    pos_x = x;
    pos_y = y;
}

void Particle::update(float timestep, int window_width, int window_height) {
    pos_x = pos_x + vel_x * timestep;
    pos_y = pos_y + vel_y * timestep;
    vel_x = vel_x - vel_x * acc_x * timestep;
    vel_y = vel_y + acc_y * timestep;

    if (pos_x < 0) {
        pos_x += window_width;
    } else if (pos_x > window_width) {
        pos_x -= window_width;
    }

    if (pos_y < 0) {
        pos_y += window_height;
    } else if (pos_y > window_height) {
        pos_y -= window_height;
    }


}

// void Particle::print_pos () {
//     cout << Particle::x_pos << " - " << Particle::y_pos << endl;
// }