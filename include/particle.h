#ifndef PARTICLE_H
#define PARTICLE_H

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;
// #define DAMPENING_COEFF 0






class Particle {

    public:
        struct vec {
            float x, y; // keep

            // make these macro #defines outside the class
            // at top of file
            float magnitude_limit { 3.0e+038 };
            float dampening_coeff { 0 };

            // move outside struct but keep public in the Particle class
            // void add (float x_to_add, float y_to_add);
            void dampening ();
            void update (vec vector, float timestep);
            void check_magnitude_limit ();
        } pos, vel, acc;
        Vec3b color;

    public:

        void initialize_vectors(
            float pos_x, float pos_y,
            float vel_x, float vel_y,
            float acc_x, float acc_y
        );
        void update (float timestep, int window_width, int window_height, int padding);
        void update_vec (Particle::vec* vec_one, Particle::vec* vec_two, float timestep);

        Particle() {}  
};

void add(Particle::vec* vec_var, float x_to_add, float y_to_add);



#endif