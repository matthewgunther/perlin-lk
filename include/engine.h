#ifndef ENGINE_H
#define ENGINE_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

#include "particle.h"
#include "flow_field.h"


#define ACC_SCALE 1000
#define FLOW_THRESHOLD 5
#define FLIP_IMAGE 1
#define LK_WINDOW_DIM 10

#define DOWNSAMPLE_SCALE 10
#define NUM_OF_BUBBLES 10000

using namespace std;
using namespace cv;


class Engine {
    public:
        Mat current_frame_color;
        Mat current_frame_float;
        Mat flow;

    private:
        VideoCapture cap;
        Mat previous_frame_float;
        Mat t_gradient;
        Mat x_gradient;
        Mat y_gradient;

        Mat x_flow;
        Mat y_flow;


        Mat x_kernel;
        Mat y_kernel;



    public:
        void compute_lk_flow ();
        void compute_t_gradient ();
        void compute_x_gradient ();
        void compute_y_gradient ();
        void destroy_all_windows ();
        char display_image (string title, Mat image, float resize_scale);
        void draw_particles (Particle particles[]);
        void initialize_kernels ();
        void initialize_lk_arrays ();
        void get_current_frame ();
        int open_camera ();
        void push_particles(Particle particles[]);
        void release_cap ();
        void store_previous_frame ();
        void visualize_lk_flow ();

        void lk_hash (
            Particle particles[], 
            unordered_map<int, vector<int>>& particle_hash,
            FlowField* p
        );


    private:
        void check_for_previous_frame ();
        Mat get_gradient_roi_vector (int r, int c, int windom_dim, Mat gradient);
};


Mat convert_color_image_to_float (Mat image);
Mat draw_color_bar (Mat image);
Vec3b get_rgb_from_hsv (float angle);
Vec3b get_color (float x, float y);
float map_atan_to_360_deg (float x, float y, float angle);
Mat resize_image (Mat image, float scale);


#endif