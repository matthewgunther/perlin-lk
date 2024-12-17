#include "bubble.h"
#include "engine.h"

#include <cmath>
#include <iostream>

Bubble::Bubble(float pos_x, float pos_y, float vel_x, float vel_y, float acc_x,
               float acc_y)
    :

      pos_(pos_x, pos_y), vel_(vel_x, vel_y), acc_(acc_x, acc_y),
      color_(255, 255, 255) {}

void Bubble::Vec::update(const Vec &vec, const float &timestep) {
  x = x + (vec.x * timestep);
  y = y + (vec.y * timestep);
}

float Bubble::get_pos_x() const { return pos_.x; };
float Bubble::get_pos_y() const { return pos_.y; };

cv::Vec3b Bubble::get_color() const { return color_; };

void Bubble::update_pos(const float &timestep) {
  vel_.update(acc_, timestep);
  pos_.update(vel_, timestep);
}

void boundary_adjust(float &pos, const int &limit) {
  if (pos > limit) {
    pos = pos - limit;
  } else if (pos < 0) {
    pos = pos + limit;
  }
}

void Bubble::check_boundaries(const int &rows, const int &cols) {
  boundary_adjust(pos_.x, cols);
  boundary_adjust(pos_.y, rows);
}

void Bubble::add_acc(const float x, const float y) {
  acc_.x += x;
  acc_.y += y;
};

void Bubble::dampen_acc() {
  float acc_mag = sqrt(pow(acc_.x, 2) + pow(acc_.y, 2));
  acc_.x +=
      (acc_.x > 0 ? -1 : 1) *
      std::min(std::abs(acc_.x), std::abs(acc_.x / acc_mag * dampen_rate_acc_));
  acc_.y +=
      (acc_.y > 0 ? -1 : 1) *
      std::min(std::abs(acc_.y), std::abs(acc_.y / acc_mag * dampen_rate_acc_));

  if (acc_mag > max_acc_) {
    acc_.x = acc_.x / acc_mag * max_acc_;
    acc_.y = acc_.y / acc_mag * max_acc_;
  }
}

void Bubble::dampen_vel() {
  float vel_mag = sqrt(pow(vel_.x, 2) + pow(vel_.y, 2));
  if (vel_mag > max_vel_) {
    vel_.x = vel_.x / vel_mag * max_vel_;
    vel_.y = vel_.y / vel_mag * max_vel_;
  }
}

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

// void check_window_bound (Particle::vec* vec_pointer, float bound_x, float
// bound_y) {
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

// void update_vec (Particle::vec* vec_one_pointer, Particle::vec*
// vec_two_pointer) {
//     vec_one_pointer->x += vec_two_pointer->x * TIMESTEP;
//     vec_one_pointer->y += vec_two_pointer->y * TIMESTEP;
// }