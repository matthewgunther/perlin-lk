#include "flow_field.h"
#include "particle.h"
#include "PerlinNoise.hpp"

#include <iostream>
#include <random>

using namespace std;


void FlowField::initialize_particles (int num_of_paricles) {
    Particle bubbles[num_of_paricles];
    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr_pos(100, 900); // define the range
    uniform_int_distribution<> distr_vec(-10, 10); // define the range


    for (int i = 0; i < num_of_paricles; i++) {
        bubbles[i].initialize_vectors(distr_pos(gen), distr_pos(gen), 0, 0, 0, 0);
        // bubbles[i].initialize_vectors(distr_pos(gen), distr_pos(gen), distr_vec(gen), distr_vec(gen), 0, 0);
        bubbles[i].vel.magnitude_limit = 60;
        bubbles[i].acc.magnitude_limit = 100;

        bubbles[i].vel.dampening_coeff = 0.125;
        bubbles[i].acc.dampening_coeff = 0.25;
    }
}


void FlowField::test () {
    cout << perlin.octave3D_01((1 * 0.1), (1 * 0.1), 0, 4) << endl;
}