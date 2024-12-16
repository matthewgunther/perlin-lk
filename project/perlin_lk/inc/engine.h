#pragma once

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

// #include "flow_field.h"

#include "bubble.h"

// // general
// #define DOWNSAMPLE_SCALE 10
// #define NUM_OF_BUBBLES 1000
#define PADDING 0

// // engine
// #define ACC_SCALE 1000
// #define FLOW_THRESHOLD 5
// #define FLIP_IMAGE 1
// #define LK_WINDOW_DIM 10

// // noise flow field
// #define FLOW_SCALE 50
// #define NOISE_ARR_SCALE 2
// #define NOISE_X_SCALAR 10
// #define NOISE_Y_SCALAR 10
// #define NOISE_Z_DELTA 0.005

// // particles
// #define TIMESTEP 0.2
// #define VEL_DAMPEN_COEFF 0.125
// #define ACC_DAMPEN_COEFF 0.25


class Engine {

    public:

         Engine(const std::uint32_t& num_bubbles);
        ~Engine();
        void run();

    private:
        // Mat flow;

        cv::VideoCapture open_camera();
        void get_frame();
        char display_image();

        void initialize_bubbles(const std::uint32_t& num_bubbles);
        void draw_bubbles() const;

        cv::VideoCapture cap_;
        cv::Mat frame_bgr_;
        std::vector<Bubble> bubbles_;






    // private:
    //     VideoCapture cap;
    //     Mat current_frame_placeholder;
    //     Mat current_frame_float; 
    //     Mat previous_frame_float;
    //     Mat t_gradient;
    //     Mat x_gradient;
    //     Mat y_gradient;
    //     Mat x_flow;
    //     Mat y_flow;
    //     Mat x_kernel;
    //     Mat y_kernel;

    // public:
    //     void check_for_previous_frame ();
    //     void compute_lk_flow ();
    //     void compute_t_gradient ();
    //     void compute_x_gradient ();
    //     void compute_y_gradient ();
    //     void destroy_all_windows ();
    //     char display_image (string title, Mat image, float resize_scale);
    //     void draw_particles (Particle particles[]);
    //     void get_current_frame ();
    //     Mat get_gradient_roi_vector (int r, int c, int windom_dim, Mat gradient);
    //     void initialize_kernels ();
    //     void initialize_lk_arrays ();
    //     void move_particles (
    //         Particle particles[], 
    //         unordered_map<int, vector<int>>& particle_hash,
    //         FlowField* p
    //     );
    //     int open_camera ();
    //     void release_cap ();
    //     void store_previous_frame ();
    //     void visualize_downsample ();
    //     void visualize_lk_flow ();
};

// Mat convert_color_image_to_float (Mat image);
// Mat draw_color_bar (Mat image);
// Vec3b get_rgb_from_hsv (float angle);
// Vec3b get_color (float x, float y);
// float map_atan_to_360_deg (float x, float y, float angle);
// Mat resize_image (Mat image, float scale);
