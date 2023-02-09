#ifndef PARTICLE_H
#define PARTICLE_H

#include <cmath>


class Particle {

    public:
        struct vec {
            float x, y;
            float magnitude_limit { 3.0e+038 };
            float dampening_coeff { 0 };
            
            void add (float x_to_add, float y_to_add);
            void dampening ();
            void update (vec vector, float timestep);
            void check_magnitude_limit ();
        } pos, vel, acc;

    public:
        void initialize_vectors(
            float pos_x, float pos_y,
            float vel_x, float vel_y,
            float acc_x, float acc_y
        );
        void update(float timestep, int window_width, int window_height, int padding);

        Particle() {}  
};

#endif