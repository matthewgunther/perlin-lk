#include <cmath>
#include <iostream>
#include <random>

#include "engine.h"
#include "opencv2/opencv.hpp"

#define _USE_MATH_DEFINES

/// @brief Flip image along horizontal
/// @param image Image to flip, flips in place
void flip_image(const cv::Mat &image) { cv::flip(image, image, 1); };

/// @brief Create color matrix from X and Y flow
/// @param x_mat X flow matrix
/// @param y_mat Y flow matrix
/// @return Matrix that is colored based on direction and magnitude of flow
cv::Mat get_color_flow_mat(const cv::Mat &x_mat, const cv::Mat &y_mat) {
  // Ensure input matrices have the same size and type
  CV_Assert(x_mat.size() == y_mat.size());
  CV_Assert(x_mat.type() == y_mat.type());

  // Create output HSV image
  cv::Mat hsv_image(x_mat.rows, x_mat.cols, CV_8UC3);

  // Iterate through each pixel
  for (int y = 0; y < x_mat.rows; y++) {
    for (int x = 0; x < x_mat.cols; x++) {
      // Get x and y values
      float x_val = x_mat.at<float>(y, x);
      float y_val = y_mat.at<float>(y, x);

      // Calculate polar coordinates
      float radius = std::sqrt(x_val * x_val + y_val * y_val) * 2;
      float angle = std::atan2(y_val, x_val);

      // Map angle to hue (0-180 in OpenCV)
      // Convert from radians to degrees, then to hue
      int hue = static_cast<int>((angle + M_PI) / (2 * M_PI) * 180);

      // Normalize hue to 0-180 range
      hue = std::max(0, std::min(179, hue));

      // Set saturation to maximum (255)
      // Map radius to value (brightness) - you might want to normalize this
      int value = static_cast<int>(radius);

      const int sat = (value > 50) ? 255 : 0;

      // Create HSV pixel
      hsv_image.at<cv::Vec3b>(y, x) = cv::Vec3b(hue, sat, 255);
    }
  }

  // Convert HSV to BGR for display
  cv::Mat bgr_image;
  cv::cvtColor(hsv_image, bgr_image, cv::COLOR_HSV2BGR);

  return bgr_image;
}

/// @brief Get flattened ROI
/// @param row Row value
/// @param col Column value
/// @param window_dim Dimension of ROI window
/// @param grad_mat Matrix of which to get ROI
/// @return Flattened ROI values
cv::Mat get_gradient_roi_vector(const int &row, const int &col,
                                const int &window_dim,
                                const cv::Mat &grad_mat) {

  cv::Mat roi = grad_mat(cv::Range(row, row + window_dim),
                         cv::Range(col, col + window_dim))
                    .clone();

  return roi.reshape(1, roi.rows * roi.cols);
}

Engine::Engine(const boost::json::value &config)
    : cap_(0), timestep_(config.at("timestep").as_double()),
      flow_mat_dim_(config.at("flow_mat_dim").as_int64()),
      flow_window_dim_(config.at("flow_window_dim").as_int64()),
      display_flow_overlay_(config.at("display_flow_overlay").as_bool()),
      perlin_z_shift_(config.at("perlin_z_shift").as_double()),
      dampen_rate_acc_{static_cast<float>(
          config.at("bubbles").at("dampen_rate_acc").as_double())},
      max_acc_{
          static_cast<float>(config.at("bubbles").at("max_acc").as_double())},
      max_vel_{
          static_cast<float>(config.at("bubbles").at("max_vel").as_double())} {
  if (!cap_.isOpened()) {
    std::cout << "Camera initialization failed." << std::endl;
  } else {
    get_frame();
    save_frame();
    initialize_bubbles(config.at("bubbles"));

    flow_x_ = cv::Mat(flow_mat_dim_, flow_mat_dim_, CV_32FC1);
    flow_y_ = cv::Mat(flow_mat_dim_, flow_mat_dim_, CV_32FC1);

    color_mat_ = cv::Mat(flow_mat_dim_, flow_mat_dim_, CV_8UC3);

    Ax_ = cv::Mat(1, pow(flow_window_dim_, 2), CV_32FC1);
    Ay_ = cv::Mat(1, pow(flow_window_dim_, 2), CV_32FC1);
    b_ = cv::Mat(1, pow(flow_window_dim_, 2), CV_32FC1);
  }
}

Engine::~Engine() {
  cv::destroyAllWindows();
  cap_.release();
}

void Engine::compute_lk_flow() {
  if (frame_gray_prev_.empty()) {
    return;
  }
  cv::Sobel(frame_gray_, grad_x_, CV_32FC1, 1, 0);
  cv::Sobel(frame_gray_, grad_y_, CV_32FC1, 0, 1);
  subtract(frame_gray_, frame_gray_prev_, grad_t_);

  for (int r = 0; r < (frame_gray_.rows / flow_window_dim_); r++) {
    for (int c = 0; c < (frame_gray_.cols / flow_window_dim_); c++) {
      const int row = r * flow_window_dim_;
      const int col = c * flow_window_dim_;
      cv::Mat Ax_ =
          get_gradient_roi_vector(row, col, flow_window_dim_, grad_x_);
      cv::Mat Ay_ =
          get_gradient_roi_vector(row, col, flow_window_dim_, grad_y_);
      cv::Mat b_ = get_gradient_roi_vector(row, col, flow_window_dim_, grad_t_);

      cv::Mat A;
      cv::hconcat(Ax_, Ay_, A);

      cv::Mat nu = (A.t() * A).inv() * A.t() * b_; // compute flow vector
      nu = -100 * nu; // make negative to flip flow direction

      flow_x_.at<float>(r, c) = nu.at<float>(0, 0);
      flow_y_.at<float>(r, c) = nu.at<float>(1, 0);
    }
  }
}

char Engine::display_image() {
  if (display_flow_overlay_) {
    cv::resize(color_mat_, color_mat_,
               cv::Size(frame_bgr_.cols, frame_bgr_.rows), 0, 0,
               cv::INTER_NEAREST);
    cv::Mat blend;
    cv::addWeighted(frame_bgr_, 0.5, color_mat_, 0.5, 0.0, blend);

    cv::imshow("BubbleBender [flow overlayed]", blend);

  } else {
    cv::imshow("BubbleBender", frame_bgr_);
  }
  return cv::waitKey(1);
}

void Engine::draw_bubbles() const {
  for (const Bubble &b : bubbles_) {
    circle(frame_bgr_,
           cv::Point(static_cast<int>(b.get_pos_x()),
                     static_cast<int>(b.get_pos_y())),
           3,
           color_mat_.at<cv::Vec3b>(
               floor(b.get_pos_y() / frame_bgr_.rows * flow_mat_dim_),
               floor(b.get_pos_x() / frame_bgr_.cols * flow_mat_dim_)),
           2);
  }
}

void Engine::get_frame() {
  cap_ >> frame_bgr_;
  flip_image(frame_bgr_);
  cvtColor(frame_bgr_, frame_gray_, cv::COLOR_BGR2GRAY);
  cv::resize(frame_gray_, frame_gray_,
             cv::Size(flow_mat_dim_ * flow_window_dim_,
                      flow_mat_dim_ * flow_window_dim_),
             0, 0, cv::INTER_LINEAR);
  frame_gray_.convertTo(frame_gray_, CV_32FC1);
  cv::GaussianBlur(frame_gray_, frame_gray_, cv::Size(5, 5), 0);
}

void Engine::initialize_bubbles(const boost::json::value &config) {
  std::random_device rd;  // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_pos_x(0, frame_bgr_.cols);
  std::uniform_int_distribution<> distr_pos_y(0, frame_bgr_.rows);

  // assign initial values
  for (int i = 0; i < config.at("num_bubbles").as_int64(); i++) {
    bubbles_.emplace_back(static_cast<float>(distr_pos_x(gen)),
                          static_cast<float>(distr_pos_y(gen)));
  }
}

void Engine::move_bubbles() {
  perlin_z += perlin_z_shift_;
  for (Bubble &b : bubbles_) {
    update_bubble_perlin(b);
    update_bubble_flow(b);
    b.dampen_acc(dampen_rate_acc_, max_acc_);
    b.dampen_vel(max_vel_);
    b.update_pos(timestep_);
    b.check_boundaries(frame_bgr_.rows, frame_bgr_.cols);
  }
}

void Engine::run() {

  if (cap_.isOpened()) {

    while (1) {
      get_frame();

      // compute optical flow + noise flow
      compute_lk_flow();
      move_bubbles();
      color_mat_ = get_color_flow_mat(flow_x_, flow_y_);
      draw_bubbles();

      // display image
      if (display_image() == 27) {
        break;
      }
      save_frame();
    }
  }
}

void Engine::save_frame() { frame_gray_prev_ = frame_gray_.clone(); }

void Engine::update_bubble_flow(Bubble &bubble) {
  const float flow_x = flow_x_.at<float>(
      floor(bubble.get_pos_y() / frame_bgr_.rows * flow_mat_dim_),
      floor(bubble.get_pos_x() / frame_bgr_.cols * flow_mat_dim_));
  const float flow_y = flow_y_.at<float>(
      floor(bubble.get_pos_y() / frame_bgr_.rows * flow_mat_dim_),
      floor(bubble.get_pos_x() / frame_bgr_.cols * flow_mat_dim_));
  bubble.add_acc(10 * flow_x, 10 * flow_y);
}

void Engine::update_bubble_perlin(Bubble &bubble) {

  // get noise
  double noise_angle =
      perlin_.octave3D_01((bubble.get_pos_x() * 10), (bubble.get_pos_y() * 10),
                          perlin_z, 4) *
      M_PI * 4;
  float flow_x = cos(noise_angle);
  float flow_y = sin(noise_angle);
  bubble.add_acc(flow_x * 0.0001, flow_y * 0.0001);
}
