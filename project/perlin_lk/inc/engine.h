#pragma once

#include <boost/json.hpp>
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "PerlinNoise.hpp"
#include "bubble.h"

/// @brief Engine to run image stream and manage bubbles
class Engine {
public:
  /// @brief Engine constructor
  /// @param config Boost JSON config
  Engine(const boost::json::value &config);

  /// @brief Engine destructor, close out OpenCV objects
  ~Engine();

  /// @brief Stream camera and interact with bubbles
  void run();

private:
  /// @brief Compute Lucas Kanade flow 2D
  void compute_lk_flow();

  /// @brief Display Image
  /// @return Key value captured by OpenCV
  char display_image();

  /// @brief Draw bubbles over image
  void draw_bubbles() const;

  /// @brief Get latest frame
  void get_frame();

  /// @brief Initialize bubbles with random position in the frame
  /// @param config Boost JSON config for bubbles
  void initialize_bubbles(const boost::json::value &config);

  /// @brief Move bubbles based on Perlin noise and LK flow
  void move_bubbles();

  /// @brief Open camera
  /// @return OpenCV VideoCapture object
  cv::VideoCapture open_camera();

  /// @brief Save current frame to previous frame
  void save_frame();

  /// @brief Update bubble based on optical flow
  /// @param bubble Bubble object to move
  void update_bubble_flow(Bubble &bubble);

  /// @brief Update bubble based on Perlin noise
  /// @param bubble Bubble object to move
  void update_bubble_perlin(Bubble &bubble);

  /// @brief OpenCV video capture object to grab frames from
  cv::VideoCapture cap_;

  /// @brief Current color frame
  cv::Mat frame_bgr_;
  /// @brief Current gray frame
  cv::Mat frame_gray_;
  /// @brief Previous gray frame
  cv::Mat frame_gray_prev_;

  /// @brief Time gradient matrix
  cv::Mat grad_t_;
  /// @brief X gradient matrix
  cv::Mat grad_x_;
  /// @brief Y gradient matrix
  cv::Mat grad_y_;

  /// @brief Optical flow for X
  cv::Mat flow_x_;
  /// @brief Optical flow for Y
  cv::Mat flow_y_;
  /// @brief Color matrix for optical flow direction
  cv::Mat color_mat_;

  /// @brief Matrix for Optical flow computation
  cv::Mat Ax_;
  /// @brief Matrix for Optical flow computation
  cv::Mat Ay_;
  /// @brief Matrix for Optical flow computation
  cv::Mat b_;
  /// @brief Concatenated Ax_ and Ay_
  cv::Mat A_;

  /// @brief Bubbles which are randomly moving and can be pushed
  std::vector<Bubble> bubbles_;
  /// @brief Timestep for physics computations
  const double timestep_;
  /// @brief Optical flow matrix dimension
  const int flow_mat_dim_;
  /// @brief Optical flow window dimension
  const int flow_window_dim_;
  /// @brief Flag to display blended image
  const bool display_flow_overlay_;

  /// @brief Perlin noise seed
  const siv::PerlinNoise::seed_type seed = 123456u;
  /// @brief Perlin noise object
  const siv::PerlinNoise perlin_{seed};
  /// @brief 3D Perlin noise value
  float perlin_z{0};
  /// @brief Delta by which to advance Perlin noise field
  const float perlin_z_shift_;

  /// @brief Maximum bubble acceleration
  const double max_acc_;
  /// @brief Accelration dampening coefficient
  const double dampen_rate_acc_;
  /// @brief Maximum bubble velocity
  const double max_vel_;
  /// @brief Threshold of flow magnitude to show color on bubbles
  const double flow_color_threshold_;
  /// @brief Blend proportion (0 - 1) of optical flow vs camera frame
  const double flow_blend_ratio_;
  /// @brief Scalar for Optical flow acceleration of bubbles
  const double flow_scalar_;
  /// @brief Scalar for Perlin Noise acceleration of bubbles
  const double perlin_scalar_;
};
