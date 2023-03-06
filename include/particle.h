#ifndef PARTICLE_H
#define PARTICLE_H

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

using namespace cv;




#define MAGNITUDE_LIMIT 3000000000

#define TIMESTEP 0.2
#define PADDING 130





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
            void check_magnitude_limit ();
        } pos, vel, acc;
        Vec3b color;

    public:
        void update (int window_width, int window_height);
        
        Particle() {}  
};

void add(Particle::vec* vec_var, float x_to_add, float y_to_add);
void update_vec (Particle::vec* vec_one, Particle::vec* vec_two);
void dampen(Particle::vec* vec_var, float dampen_coeff);
// void check_magnitude_limit(Particle::vec* vec_var);

#endif