#include "engine.h"

#include "opencv2/opencv.hpp"
#include <iostream>

#define _USE_MATH_DEFINES
#include <cmath>

#include <random>

// struct frame_counter {
//     public:
//         int frame_counter { 0 };
//         int64_t start_tick { getTickCount() };

//     void print_fps () {
//         if (frame_counter == 30) {   // Calculate and print FPS every 30
//         frames
//             int64_t end_tick = getTickCount();
//             double fps = frame_counter / ((end_tick - start_tick) /
//             getTickFrequency()); cout << "FPS: " << fps << endl;

//             // Reset counters
//             start_tick = end_tick;
//             frame_counter = 0;
//         }
//     }
// };

// void get_gradient_roi_vector (cv::Mat& mat, const int& r, const int& c, const
// int& window_dim, const cv::Mat& gradient) {

//     cv::Mat roi =  gradient(
//         cv::Range(r - window_dim, r + window_dim),
//         cv::Range(c - window_dim, c + window_dim)
//     ).clone();
//     for (int c = 0; c < mat.cols; c++) {
//         mat.at<float>(0, c) = roi.at<float>(
//             c % roi.cols,
//             floor(c / roi.cols)
//         );
//     }

//     // return roi.reshape(1, roi.rows * roi.cols);
// }

cv::Mat get_gradient_roi_vector(const int &r, const int &c,
                                const int &window_dim,
                                const cv::Mat &gradient) {

  cv::Mat roi = gradient(
                    // cv::Range(r - window_dim, r + window_dim),
                    // cv::Range(c - window_dim, c + window_dim)
                    cv::Range(r, r + window_dim), cv::Range(c, c + window_dim))
                    .clone();

  return roi.reshape(1, roi.rows * roi.cols);
}

/// @brief Flip image along horizontal
void flip_image(const cv::Mat &image) { cv::flip(image, image, 1); };

/// @brief Run program
void Engine::run() {

  if (cap_.isOpened()) {

    while (1) {
      get_frame();

      // // compute optical flow + noise flow
      compute_lk_flow();
      move_bubbles();
      draw_bubbles();

      // display image
      // TODO(mjg): remove user input
      if (display_image() == 27) {
        break;
      }
      save_frame();
    }
  }
}

Engine::Engine(const boost::json::value &config)
    : cap_(0), timestep_(config.at("timestep").as_double()),
      flow_window_dim_(config.at("flow_window_dim").as_int64()),
      display_flow_overlay_(config.at("display_flow_overlay").as_bool()) {
  if (!cap_.isOpened()) {
    std::cout << "Camera initialization failed." << std::endl;
  } else {
    get_frame();
    save_frame();
    std::cout << "Frame: " << frame_bgr_.rows << " " << frame_bgr_.cols
              << std::endl;
    initialize_bubbles(config.at("num_bubbles").as_int64());
    flow_x_ = cv::Mat(10, 10, CV_32FC1);
    flow_y_ = cv::Mat(10, 10, CV_32FC1);

    Ax_ = cv::Mat(1, pow(2 * flow_window_dim_, 2), CV_32FC1);
    Ay_ = cv::Mat(1, pow(2 * flow_window_dim_, 2), CV_32FC1);
    b_ = cv::Mat(1, pow(2 * flow_window_dim_, 2), CV_32FC1);
  }
}

/// @brief Close down OpenCV objects
Engine::~Engine() {
  cv::destroyAllWindows();
  cap_.release();
}

void Engine::initialize_bubbles(const std::uint32_t &num_bubbles) {
  std::random_device rd;  // obtain a random number from hardware
  std::mt19937 gen(rd()); // seed the generator
  std::uniform_int_distribution<> distr_pos_x(PADDING,
                                              frame_bgr_.cols - PADDING);
  std::uniform_int_distribution<> distr_pos_y(PADDING,
                                              frame_bgr_.rows - PADDING);
  std::uniform_int_distribution<> distr_vec(
      -10, 10); // define the range for velocities

  // assign initial values
  for (int i = 0; i < num_bubbles; i++) {
    bubbles_.emplace_back(static_cast<float>(distr_pos_x(gen)),
                          static_cast<float>(distr_pos_y(gen)),
                          static_cast<float>(distr_pos_x(gen)) / -10,
                          static_cast<float>(distr_pos_y(gen)) / -10,
                          // 2.0f,
                          // 5.0f,
                          0.0f, 0.0f);
  }
}

/// @brief Read frame from camera
void Engine::get_frame() {
  cap_ >> frame_bgr_;
  flip_image(frame_bgr_);
  cvtColor(frame_bgr_, frame_gray_, cv::COLOR_BGR2GRAY);
  cv::resize(frame_gray_, frame_gray_, cv::Size(100, 100), 0, 0,
             cv::INTER_LINEAR);
  frame_gray_.convertTo(frame_gray_, CV_32FC1);
  cv::GaussianBlur(frame_gray_, frame_gray_, cv::Size(5, 5), 0);
}

cv::Mat convertToPolarImage(const cv::Mat &x_matrix, const cv::Mat &y_matrix) {
  // Ensure input matrices have the same size and type
  CV_Assert(x_matrix.size() == y_matrix.size());
  CV_Assert(x_matrix.type() == y_matrix.type());

  // Create output HSV image
  cv::Mat hsv_image(x_matrix.rows, x_matrix.cols, CV_8UC3);

  // Iterate through each pixel
  for (int y = 0; y < x_matrix.rows; y++) {
    for (int x = 0; x < x_matrix.cols; x++) {
      // Get x and y values
      float x_val = x_matrix.at<float>(y, x);
      float y_val = y_matrix.at<float>(y, x);

      // Calculate polar coordinates
      float radius = std::sqrt(x_val * x_val + y_val * y_val) * 2;
      float angle = std::atan2(y_val, x_val);

      radius = pow(radius, 2);

      // std::cout << "radius" << radius << std::endl;

      // Map angle to hue (0-180 in OpenCV)
      // Convert from radians to degrees, then to hue
      int hue = static_cast<int>((angle + M_PI) / (2 * M_PI) * 180);

      // Normalize hue to 0-180 range
      hue = std::max(0, std::min(179, hue));

      // Set saturation to maximum (255)
      // Map radius to value (brightness) - you might want to normalize this
      int value = static_cast<int>(std::min(255.0f, radius));

      // Create HSV pixel
      hsv_image.at<cv::Vec3b>(y, x) = cv::Vec3b(hue, 255, value);
    }
  }

  // Convert HSV to BGR for display
  cv::Mat bgr_image;
  cv::cvtColor(hsv_image, bgr_image, cv::COLOR_HSV2BGR);

  return bgr_image;
}

/// @brief Display image
char Engine::display_image() {
  if (display_flow_overlay_) {
    cv::Mat flow = convertToPolarImage(flow_x_, flow_y_);
    cv::resize(flow, flow, cv::Size(frame_bgr_.cols, frame_bgr_.rows), 0, 0,
               cv::INTER_NEAREST);
    cv::Mat blend;
    cv::addWeighted(frame_bgr_, 0.5, flow, 0.5, 0.0, blend);

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
           3, b.get_color(), 2);
  }
}

void Engine::perlin_update(Bubble &bubble) {

  // get noise
  double noise_angle =
      perlin_.octave3D_01((bubble.get_pos_x() * 10), (bubble.get_pos_y() * 10),
                          perlin_z, 4) *
      M_PI * 4;
  float flow_x = cos(noise_angle);
  float flow_y = sin(noise_angle);
  bubble.add_acc(flow_x, flow_y);
}

void Engine::flow_update(Bubble &bubble) {
  bubble.add_acc(
      10 * flow_x_.at<float>(floor(bubble.get_pos_y() / frame_bgr_.rows * 10),
                             floor(bubble.get_pos_x() / frame_bgr_.cols * 10)),
      10 * flow_y_.at<float>(floor(bubble.get_pos_y() / frame_bgr_.rows * 10),
                             floor(bubble.get_pos_x() / frame_bgr_.cols * 10)));
}

void Engine::move_bubbles() {
  perlin_z += 0.005;
  for (Bubble &b : bubbles_) {
    perlin_update(b);
    flow_update(b);
    b.dampen_acc();
    b.dampen_vel();
    b.update_pos(timestep_);
    b.check_boundaries(frame_bgr_.rows, frame_bgr_.cols);
  }
}

void Engine::compute_lk_flow() {
  if (frame_gray_prev_.empty()) {
    return;
  }
  std::cout << "compute" << std::endl;
  cv::Sobel(frame_gray_, grad_x_, CV_32FC1, 1, 0);
  cv::Sobel(frame_gray_, grad_y_, CV_32FC1, 0, 1);
  subtract(frame_gray_, frame_gray_prev_, grad_t_);

  // for (int r = flow_window_dim_; r < (frame_gray_.rows - flow_window_dim_);
  // r++) {
  //     for (int c = flow_window_dim_; c < (frame_gray_.cols -
  //     flow_window_dim_); c++) {

  for (int r = 0; r < (frame_gray_.rows / 10); r++) {
    for (int c = 0; c < (frame_gray_.cols / 10); c++) {
      // std::cout << "";
      // get_gradient_roi_vector(Ax_, r, c, flow_window_dim_, grad_x_);
      // get_gradient_roi_vector(Ay_, r, c, flow_window_dim_, grad_y_);
      // get_gradient_roi_vector(b_, r, c, flow_window_dim_, grad_t_);
      cv::Mat Ax_ =
          get_gradient_roi_vector(r * 10, c * 10, flow_window_dim_, grad_x_);
      cv::Mat Ay_ =
          get_gradient_roi_vector(r * 10, c * 10, flow_window_dim_, grad_y_);
      cv::Mat b_ =
          get_gradient_roi_vector(r * 10, c * 10, flow_window_dim_, grad_t_);

      cv::Mat A;
      cv::hconcat(Ax_, Ay_, A);

      cv::Mat nu = (A.t() * A).inv() * A.t() * b_; // compute flow vector
      nu = -10 * nu; // make negative to flip flow direction

      flow_x_.at<float>(r, c) = nu.at<float>(0, 0);
      flow_y_.at<float>(r, c) = nu.at<float>(1, 0);
    }
  }
  std::cout << "loop" << std::endl;
}

void Engine::save_frame() { frame_gray_prev_ = frame_gray_.clone(); }

// // -------------------------------------------------------------

// void Engine::run() {

//     if (en.open_camera() == 0) {

//         en.get_current_frame();
//         en.initialize_lk_arrays();
//         en.initialize_kernels();

//         unordered_map<int, vector<int>> bubble_hash;

//         // initialize particle properties
//         Particle bubbles[NUM_OF_BUBBLES];
//         FlowField ff;
//         ff.initialize_particles(
//             bubbles,
//             bubble_hash,
//             en.current_frame_color.rows,
//             en.current_frame_color.cols
//         );

//         frame_counter fc;

//         while (1) {
//             // print frames per second
//             fc.frame_counter++;
//             fc.print_fps();

//             // compute optical flow + noise flow
//             en.get_current_frame();
//             en.compute_t_gradient();
//             en.compute_x_gradient();
//             en.compute_y_gradient();
//             en.move_particles(bubbles, bubble_hash, &ff);
//             en.draw_particles(bubbles);

//             // display image
//             char key_press = en.display_image("Bubble Bender",
//             en.current_frame_color, 0); if (key_press==27) {
//                 en.destroy_all_windows();
//                 en.release_cap();
//                 break;
//             }
//             en.store_previous_frame();
//         }
//     }

// };

// // helper functions
// Mat convert_color_image_to_float(Mat image) {
//     cvtColor(image, image, COLOR_BGR2GRAY);
//     image.convertTo(image, CV_32FC1);
//     return image;
// }

// Mat draw_color_bar (Mat image) {
//     for (int r=0; r < int(image.rows / 20); r++) {
//         for (int c=0; c < int(image.cols / 8); c++) {
//             image.at<Vec3b>(r, c) = get_rgb_from_hsv((float)(int(c * 8 * 360
//             / image.cols)));
//         }
//     }
//     return image;
// }

// Vec3b get_color(float x, float y) {
//     float angle = atanf32(y / x) * 180 / M_PI;
//     angle = map_atan_to_360_deg(x, y, angle); // maps arctan output to 360
//     degrees Vec3b color = get_rgb_from_hsv(angle); return color;
// }

// Vec3b get_rgb_from_hsv(float angle) {

//     Vec3b color;

//     float H = angle / 60.0f;
//     float V = 1.0f;
//     float S = 1.0f;

//     float C = V * S;
//     int X_rgb = (int)(C * (1 - fabs(fmod(H, 2.0f) - 1)) * 255.0f);
//     int C_rgb = (int)(C * 255.0f);

//     if (H >= 0 & H < 1) {
//         color[0] = C_rgb;
//         color[1] = X_rgb;
//         color[2] = 0;
//     } else if (H >= 1 & H < 2) {
//         color[0] = X_rgb;
//         color[1] = C_rgb;
//         color[2] = 0;
//     } else if (H >= 2 & H < 3) {
//         color[0] = 0;
//         color[1] = C_rgb;
//         color[2] = X_rgb;
//     } else if (H >= 3 & H < 4) {
//         color[0] = 0;
//         color[1] = X_rgb;
//         color[2] = C_rgb;
//     } else if (H >= 4 & H < 5) {
//         color[0] = X_rgb;
//         color[1] = 0;
//         color[2] = C_rgb;
//     } else if (H >= 5 & H < 6) {
//         color[0] = C_rgb;
//         color[1] = 0;
//         color[2] = X_rgb;
//     }

//     return color;
// }

// float map_atan_to_360_deg(float x, float y, float angle) {
//     if (x > 0) {
//         if (angle < 0) {
//             angle += 360.0f;
//         }
//     } else {
//         angle += 180.0f;
//     }
//     return angle;
// }

// Mat resize_image (Mat image, float scale) {
//     resize(image, image, Size(image.cols / scale, image.rows / scale));
//     return image;
// }

// // module functions
// void Engine::check_for_previous_frame () {
//     if (previous_frame_float.empty()) {
//         previous_frame_float = current_frame_float.clone();
//     }
// }

// void Engine::compute_lk_flow () {
//     for (int r = LK_WINDOW_DIM; r < (current_frame_float.rows -
//     LK_WINDOW_DIM); r++) {
//         for (int c = LK_WINDOW_DIM; c < (current_frame_float.cols -
//         LK_WINDOW_DIM); c++) {

//             Mat Ax = get_gradient_roi_vector(r, c, LK_WINDOW_DIM,
//             x_gradient); Mat Ay = get_gradient_roi_vector(r, c,
//             LK_WINDOW_DIM, y_gradient); Mat b = get_gradient_roi_vector(r, c,
//             LK_WINDOW_DIM, t_gradient);

//             Mat A;
//             hconcat(Ax, Ay, A);

//             Mat nu = (A.t() * A).inv() * A.t() * b; // compute flow vector
//             nu = -10 * nu; // make negative to flip flow direction

//             x_flow.at<float>(r, c) = nu.at<float>(0, 0);
//             y_flow.at<float>(r, c) = nu.at<float>(1, 0);
//         }
//     }
// }

// void Engine::compute_t_gradient () {
//     subtract(current_frame_float, previous_frame_float, t_gradient);
// }

// void Engine::compute_x_gradient () {
//     filter2D(
//         current_frame_float,
//         x_gradient,
//         -1 ,
//         x_kernel,
//         Point(-1, -1),
//         0,
//         BORDER_DEFAULT
//     );
// }

// void Engine::compute_y_gradient () {
//     filter2D(
//         current_frame_float,
//         y_gradient,
//         -1 ,
//         y_kernel,
//         Point(-1, -1),
//         0,
//         BORDER_DEFAULT
//     );
// }

// void Engine::destroy_all_windows () {
//     destroyAllWindows();
// }

// char Engine::display_image (string title, Mat image, float resize_scale=0) {

//     if (resize_scale != 0) {
//         image = resize_image(image, (1 / resize_scale));
//     }
//     imshow(title, image);

//     char key_press = waitKey(1);
//     return key_press;
// }

// void Engine::draw_particles (Particle particles[]) {
//     for (int i = 0; i < NUM_OF_BUBBLES; i++) {
//         // x is columns, y is rows
//         circle(current_frame_color, Point(int(particles[i].pos.x),
//         int(particles[i].pos.y)), 3, particles[i].color, 2);
//     }
// }

// void Engine::initialize_kernels () {
//     x_kernel = Mat(1, 3, CV_32F);
//     x_kernel.at<float>(0, 0) = -1.0f;
//     x_kernel.at<float>(0, 1) = 0.0f;
//     x_kernel.at<float>(0, 2) = 1.0f;

//     y_kernel = Mat(3, 1, CV_32F);
//     y_kernel.at<float>(0, 0) = -1.0f;
//     y_kernel.at<float>(1, 0) = 0.0f;
//     y_kernel.at<float>(2, 0) = 1.0f;
// }

// void Engine::initialize_lk_arrays () {
//     x_flow = Mat::ones(current_frame_float.rows, current_frame_float.cols,
//     CV_32FC1); y_flow = Mat::ones(current_frame_float.rows,
//     current_frame_float.cols, CV_32FC1);
// }

// void Engine::get_current_frame () {
//     cap >> current_frame_color;

//     if (FLIP_IMAGE) {
//         flip(current_frame_color, current_frame_color, 1);
//     }
//     current_frame_placeholder =
//     convert_color_image_to_float(current_frame_color);
//     current_frame_placeholder = resize_image(current_frame_placeholder,
//     DOWNSAMPLE_SCALE);

//     int top = LK_WINDOW_DIM, bottom = 50, left = 50, right = 50;

//     copyMakeBorder(
//         current_frame_placeholder,
//         current_frame_float,
//         LK_WINDOW_DIM,
//         LK_WINDOW_DIM,
//         LK_WINDOW_DIM,
//         LK_WINDOW_DIM,
//         BORDER_CONSTANT,
//         Scalar(0, 0, 0)
//     );

//     check_for_previous_frame();
// }

// Mat Engine::get_gradient_roi_vector (int r, int c, int window_dim, Mat
// gradient) {
//     Mat roi = gradient(
//         Range(r - window_dim, r + window_dim),
//         Range(c - window_dim, c + window_dim)
//     ).clone();

//     roi = roi.reshape(1, roi.rows * roi.cols);
//     return roi;
// }

// void Engine::move_particles (
//     Particle particles[],
//     unordered_map<int, vector<int>>& particle_hash,
//     FlowField* p
//     ) {

//     p->perlin_z += NOISE_Z_DELTA; // advance perlin noise field
//     for (auto const& pair : particle_hash) {

//         int key = pair.first;
//         // coordinates in downsampled space
//         int x = key % (current_frame_float.cols - LK_WINDOW_DIM*2);
//         int y = (key - x) / (current_frame_float.cols - LK_WINDOW_DIM*2);

//         // get noise
//         double noise_angle = p->perlin.octave3D_01(
//                 (x * NOISE_X_SCALAR),
//                 (y * NOISE_Y_SCALAR),
//                 p->perlin_z,
//                 4
//             ) * M_PI * 4;
//         float flow_x = cos(noise_angle);
//         float flow_y = sin(noise_angle);

//         // calculate lk flow
//         Mat Ax = get_gradient_roi_vector(
//             y + LK_WINDOW_DIM,
//             x + LK_WINDOW_DIM,
//             LK_WINDOW_DIM,
//             x_gradient
//         );
//         Mat Ay = get_gradient_roi_vector(
//             y + LK_WINDOW_DIM,
//             x + LK_WINDOW_DIM,
//             LK_WINDOW_DIM,
//             y_gradient
//         );
//         Mat b = get_gradient_roi_vector(
//             y + LK_WINDOW_DIM,
//             x + LK_WINDOW_DIM,
//             LK_WINDOW_DIM,
//             t_gradient
//         );

//         Mat A;
//         hconcat(Ax, Ay, A);

//         Mat nu = (A.t() * A).inv() * A.t() * b; // compute flow vector
//         nu = -10 * nu; // make negative to flip flow direction

//         float add_x = nu.at<float>(0, 0);
//         float add_y = nu.at<float>(1, 0);

//         // iterate through particles
//         for (auto const& i : pair.second) {
//             // flow threshold reached, color with direction and add
//             acceleration if (sqrtf32(pow(add_x, 2) + pow(add_y, 2)) >
//             FLOW_THRESHOLD) {
//                 float angle = atanf32(add_y / add_x) * 180 / M_PI;
//                 angle = map_atan_to_360_deg(add_x, add_y, angle); // maps
//                 arctan output to 360 degrees add(&particles[i].acc, (add_x *
//                 ACC_SCALE), (add_y * ACC_SCALE)); particles[i].color =
//                 get_rgb_from_hsv(angle);
//             }
//             // threshold not reached, particle remains white
//             else {
//                 Vec3b color;
//                 color[0] = 255;
//                 color[1] = 255;
//                 color[2] = 255;
//                 particles[i].color = color;
//             }

//             // add optical flow acceleration, user "pushing" particles
//             add(&particles[i].acc, (flow_x * FLOW_SCALE), (flow_y *
//             FLOW_SCALE));
//             // update physics
//             update_vec(&particles[i].vel, &particles[i].acc);
//             update_vec(&particles[i].pos, &particles[i].vel);
//             // keep particles in window
//             check_window_bound(&particles[i].pos, current_frame_color.cols,
//             current_frame_color.rows);
//             // keep particles within speed range
//             check_magnitude_limit(&particles[i].vel);
//             check_magnitude_limit(&particles[i].acc);
//             // dampen the motion of particles after push
//             dampen(&particles[i].vel, VEL_DAMPEN_COEFF);
//             dampen(&particles[i].acc, ACC_DAMPEN_COEFF);
//         }
//     }

//     particle_hash.clear();
//     for (int i = 0; i < NUM_OF_BUBBLES; i++) {
//         // linear index for each point
//         int key = floor(particles[i].pos.y / DOWNSAMPLE_SCALE)
//             * floor(current_frame_color.cols / DOWNSAMPLE_SCALE)
//             + floor(particles[i].pos.x / DOWNSAMPLE_SCALE);

//         if (particle_hash.find(key) == particle_hash.end()) {
//             // not found
//             particle_hash[key] = {i};
//         } else {
//             particle_hash[key].push_back(i);
//         }
//     }
// }

// int Engine::open_camera () {
//     cout << "Opening camera...";
//     VideoCapture cap_init(0, CAP_V4L2);
//     if (!cap_init.isOpened()) {
//         cout << "FAILED camera open" << endl;
//         return 1;
//     }
//     cout << "SUCCESSFUL camera open" << endl;
//     cap = cap_init;
//     return 0;
// }

// void Engine::release_cap () {
//     cap.release();
// }

// void Engine::store_previous_frame () {
//     previous_frame_float = current_frame_float.clone();
// }

// // to view how particles are grouped from downsample
// void Engine::visualize_downsample () {
//     // rows
//     for (float r=1; r < current_frame_float.rows; r++) {
//         int row_line = floor(r / current_frame_float.rows *
//         current_frame_color.rows); line(
//             current_frame_color,
//             Point(0, row_line),
//             Point(current_frame_color.cols, row_line),
//             Scalar(0, 0, 100),
//             2
//         );
//     }
//     // columns
//     for (float c=1; c < current_frame_float.cols; c++) {
//         int col_line = floor(c / current_frame_float.cols *
//         current_frame_color.cols); line(
//             current_frame_color,
//             Point(col_line, 0),
//             Point(col_line, current_frame_color.rows),
//             Scalar(0, 0, 100),
//             2
//         );
//     }
// }

// void Engine::visualize_lk_flow () {
// reset background
//     flow = Mat::zeros(current_frame_float.rows, current_frame_float.cols,
//     CV_8UC3); int flow_threshold { 10 }; for (int r = 0; r <
//     current_frame_float.rows; r++) {
//         for (int c = 0; c < current_frame_float.cols; c++) {
//             float x = x_flow.at<float>(r, c);
//             float y = y_flow.at<float>(r, c);
//             if (sqrtf32(pow(x, 2) + pow(y, 2)) > flow_threshold) {
//                 Vec3b color = get_color(x, y);
//                 flow.at<Vec3b>(r, c) = color;
//             }
//         }
//     }
//     flow = draw_color_bar(flow);
// }