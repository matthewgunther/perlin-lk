#include <cmath>
#include <iostream>

#include "bubble.h"
#include "engine.h"

/// @brief Adjust value to be within limits
/// @param val Value
/// @param limit Limit
void boundary_adjust(float &val, const int &limit) {
  if (val > limit) {
    val = val - limit;
  } else if (val < 0) {
    val = val + limit;
  }
}

Bubble::Bubble(float pos_x, float pos_y) : pos_(pos_x, pos_y) {}

void Bubble::add_acc(const float x, const float y) {
  acc_.x += x;
  acc_.y += y;
}

void Bubble::check_boundaries(const int &rows, const int &cols) {
  boundary_adjust(pos_.x, cols);
  boundary_adjust(pos_.y, rows);
}

void Bubble::dampen_acc(const float &dampen_rate_acc, const float &max_acc) {
  float acc_mag = sqrt(pow(acc_.x, 2) + pow(acc_.y, 2));
  acc_.x +=
      (acc_.x > 0 ? -1 : 1) *
      std::min(std::abs(acc_.x), std::abs(acc_.x / acc_mag * dampen_rate_acc));
  acc_.y +=
      (acc_.y > 0 ? -1 : 1) *
      std::min(std::abs(acc_.y), std::abs(acc_.y / acc_mag * dampen_rate_acc));

  if (acc_mag > max_acc) {
    acc_.x = acc_.x / acc_mag * max_acc;
    acc_.y = acc_.y / acc_mag * max_acc;
  }
}

void Bubble::dampen_vel(const float &max_vel) {
  float vel_mag = sqrt(pow(vel_.x, 2) + pow(vel_.y, 2));
  if (vel_mag > max_vel) {
    vel_.x = vel_.x / vel_mag * max_vel;
    vel_.y = vel_.y / vel_mag * max_vel;
  }
}

float Bubble::get_pos_x() const { return pos_.x; };

float Bubble::get_pos_y() const { return pos_.y; };

void Bubble::update_pos(const float &timestep) {
  vel_.update(acc_, timestep);
  pos_.update(vel_, timestep);
}

void Bubble::Vec::update(const Vec &vec, const float &timestep) {
  x = x + (vec.x * timestep);
  y = y + (vec.y * timestep);
}
