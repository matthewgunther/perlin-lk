#ifndef ENGINE_H
#define ENGINE_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Engine {
    public:
        Mat current_frame_float;
        Mat current_frame_color;
    
    private:
        VideoCapture cap;
        Mat previous_frame_float;
        Mat t_gradient;
        Mat x_gradient;
        // Mat x_kernel(1, 3, CV_32F);
        
        // Mat y_kernel(3, 1, CV_32F);
        // y_kernel.at<float>(0, 0) = -1.0f;
        // y_kernel.at<float>(1, 0) = 0.0f;
        // y_kernel.at<float>(2, 0) = 1.0f;

    public:
        int open_camera ();
        void get_current_frame (int flip_image = 0);
        char display_image (string title, Mat image);
        void compute_t_gradient ();
        // void downsample
        void store_previous_frame ();
        void compute_x_gradient ();

    private:
        void check_for_previous_frame ();
};

#endif