#ifndef FLOW_FIELD_H
#define FLOW_FIELD_H

#include <iostream>
#include "PerlinNoise.hpp"

using namespace std;
using namespace siv;

class FlowField {
    private:
        const PerlinNoise::seed_type seed = 123456u;
        const PerlinNoise perlin { seed };

    public:
        void initialize_particles (int num_of_particles);
        void test ();
};

#endif