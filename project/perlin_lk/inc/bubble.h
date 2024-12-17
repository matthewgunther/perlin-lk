#pragma once

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

class Bubble {
public:
  Bubble(float pos_x, float pos_y, float vel_x, float vel_y, float acc_x,
         float acc_y);
  float get_pos_x() const;
  float get_pos_y() const;
  cv::Vec3b get_color() const;

  void update_pos(const float &timestep);
  void check_boundaries(const int &rows, const int &cols);

  void add_acc(const float x, const float y);
  void dampen_vel();
  void dampen_acc();

private:
  struct Vec {
    Vec(float x_val, float y_val) : x(x_val), y(y_val) {}

    float x, y; // keep

    /// @brief Update vector with derivative and timestep
    void update(const Vec &vec, const float &timestep);

    // float magnitude_limit { 3.0e+038 };
    // float dampening_coeff { 0 };
  } pos_, vel_, acc_;

  cv::Vec3b color_;

  const float dampen_rate_acc_{2};
  const float max_vel_{8};
  const float max_acc_{10};
};

// void add(Particle::vec* vec_pointer, float x_to_add, float y_to_add);
// void check_magnitude_limit (Particle::vec* vec_pointer);
// void check_window_bound (Particle::vec* vec_pointer, float bound_x, float
// bound_y); void dampen (Particle::vec* vec_pointer, float dampen_coeff); void
// update_vec (Particle::vec* vec_one_pointer, Particle::vec* vec_two_pointer);
