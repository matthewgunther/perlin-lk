#ifndef PARTICLE_H
#define PARTICLE_H

#include <cmath>


// float check_if_nan(float value) {
//     if (isnanf(value)) {
//         return 0;
//     }
//     return value;
// }



class Particle {

    public:

        struct vec {
            float x, y;
            float magnitude_limit { 3.0e+038 };
            
            void add (float x_to_add, float y_to_add) {
                if (isnanf(x_to_add) || abs(x_to_add) <= 1 || abs(x_to_add) > 100)
                    x_to_add = 0;
                if (isnanf(y_to_add) || abs(y_to_add) <= 1 || abs(y_to_add) > 100)
                    y_to_add = 0;

                x += x_to_add;
                y += y_to_add;
                check_magnitude_limit();
            }

            void update (vec vector, float timestep) {
                x += vector.x * timestep;
                y += vector.y * timestep;
            }

            void check_magnitude_limit () {
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

        } pos, vel, acc;


    public:

        void initialize_vectors(
            float pos_x, float pos_y,
            float vel_x, float vel_y,
            float acc_x, float acc_y
        );
        void update(float timestep, int window_width, int window_height);

        Particle() {}  
};

#endif