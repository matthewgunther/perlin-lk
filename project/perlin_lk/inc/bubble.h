#pragma once

#include <cmath>
#include <opencv4/opencv2/opencv.hpp>

/// @brief Bubble to display floating around in 2D
class Bubble {
public:
  /// @brief Bubble constructor
  /// @param pos_x Initial X position
  /// @param pos_y Initial Y position
  Bubble(float pos_x, float pos_y);

  /// @brief Add values to acceleration
  /// @param x X value
  /// @param y Y value
  void add_acc(const float x, const float y);

  /// @brief Wrap bubble if out of frame
  /// @param rows Row limit
  /// @param cols Columns limit
  void check_boundaries(const int &rows, const int &cols);

  /// @brief Dampen velocity
  /// @param max_vel Maximum bubble velocity
  void dampen_vel(const float &max_vel);

  /// @brief Dampen acceleration
  /// @param dampen_rate_acc Rate by which to slow acceleration magnitude
  /// @param max_acc Maximum bubble acceleration
  void dampen_acc(const float &dampen_rate_acc, const float &max_acc);

  /// @brief Get X position
  /// @return X position as float
  float get_pos_x() const;

  /// @brief Get Y position
  /// @return Y position as float
  float get_pos_y() const;

  /// @brief Advance physics by timestep
  /// @param timestep Float of time delta
  void update_pos(const float &timestep);

private:
  /// @brief Struct for bubble position, velocity, and acceleration
  struct Vec {
    /// @brief Vec constructor
    /// @param x_val Initial X value
    /// @param y_val Initial Y value
    Vec(float x_val, float y_val) : x(x_val), y(y_val) {}

    /// @brief Coordinate values
    float x, y;

    /// @brief Update vector with derivative and timestep
    /// @param vec Derivative vector
    /// @param timstep Time delta
    void update(const Vec &vec, const float &timestep);

    /// @brief position, velocity, acceleration
  } pos_, vel_{0.0f, 0.0f}, acc_{0.0f, 0.0f};
};
