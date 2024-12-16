#include "engine.h"
#include "bubble.h"

#include <iostream>

// using namespace std;



Bubble::Bubble(
    float pos_x, float pos_y,
    float vel_x, float vel_y,
    float acc_x, float acc_y
):

pos_(pos_x, pos_y),
vel_(vel_x, vel_y),
acc_(acc_x, acc_y),
color_(255, 255, 255)
{

}

        float Bubble::get_pos_x() const{
            return pos_.x;
        };
        float Bubble::get_pos_y() const {
            return pos_.y;
        };


        cv::Vec3b Bubble::get_color() const
        {return color_;};



// void add (Particle::vec* vec_pointer, float x_to_add, float y_to_add) {

//     if (isnanf(x_to_add) || abs(x_to_add) <= 1) {
//         x_to_add = 0;
//     }
//     if (isnanf(y_to_add) || abs(y_to_add) <= 1) {
//         y_to_add = 0;
//     }
//     vec_pointer->x += x_to_add;
//     vec_pointer->y += y_to_add;
//     check_magnitude_limit(vec_pointer);
// }

// void check_magnitude_limit (Particle::vec* vec_pointer) {
//     // check x
//     if (fabs(vec_pointer->x) > vec_pointer->magnitude_limit) {
//         if (vec_pointer->x > vec_pointer->magnitude_limit) {
//             vec_pointer->x = vec_pointer->magnitude_limit;
//         } else {
//             vec_pointer->x = -1 * vec_pointer->magnitude_limit;
//         }
//     }
//     // check y
//     if (fabs(vec_pointer->y) > vec_pointer->magnitude_limit) {
//         if (vec_pointer->y > vec_pointer->magnitude_limit) {
//             vec_pointer->y = vec_pointer->magnitude_limit;
//         } else {
//             vec_pointer->y = -1 * vec_pointer->magnitude_limit;
//         }
//     }
// }

// void check_window_bound (Particle::vec* vec_pointer, float bound_x, float bound_y) {
//     // check x
//     if (vec_pointer->x < PADDING) {
//         vec_pointer->x += (bound_x - PADDING * 2);
//     } else if (vec_pointer->x > (bound_x - PADDING)) {
//         vec_pointer->x -= (bound_x - PADDING * 2);
//     }
//     // check y
//     if (vec_pointer->y < PADDING) {
//         vec_pointer->y += (bound_y - PADDING * 2);
//     } else if (vec_pointer->y > (bound_y - PADDING)) {
//         vec_pointer->y -= (bound_y - PADDING * 2);
//     }
// }

// void dampen (Particle::vec* vec_pointer, float dampen_coeff) {
//     vec_pointer->x = vec_pointer->x * dampen_coeff;
//     vec_pointer->y = vec_pointer->y * dampen_coeff;
// }

// void update_vec (Particle::vec* vec_one_pointer, Particle::vec* vec_two_pointer) {
//     vec_one_pointer->x += vec_two_pointer->x * TIMESTEP;
//     vec_one_pointer->y += vec_two_pointer->y * TIMESTEP;
// }