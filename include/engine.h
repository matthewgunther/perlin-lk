#ifndef ENGINE_H
#define ENGINE_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "particle.h"

#define ACC_SCALE 1000
#define FLOW_THRESHOLD 5
#define FLIP_IMAGE 1
#define LK_WINDOW_DIM 10

using namespace std;
using namespace cv;

class Engine {
    public:
        Mat current_frame_color;
        Mat flow;
    
    private:
        VideoCapture cap;
        Mat current_frame_float;
        Mat previous_frame_float;
        Mat t_gradient;
        Mat x_gradient;
        Mat y_gradient;
        Mat x_flow;
        Mat y_flow;


    public:
        int open_camera ();
        void get_current_frame (float downsample_scale = 1);
        char display_image (string title, Mat image, float downsample_scale = 1);
        void compute_t_gradient ();
        void compute_x_gradient ();
        void compute_y_gradient ();
        void compute_lk_flow ();
        void visualize_lk_flow ();

        void store_previous_frame ();
        void destroy_all_windows ();
        void release_cap ();

        void draw_particles(Particle particles[], int num_of_particles);

        void push_particles(Particle particles[], int num_of_particles, float downsample_scale);

    private:
        void check_for_previous_frame ();
        Mat get_gradient_roi_vector (int r, int c, int windom_dim, Mat gradient);
};

#endif