#ifndef ENGINE_H
#define ENGINE_H

#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;

class Engine {
    public:
        Mat current_frame_bw;
        Mat current_frame_color;
    
    private:
        VideoCapture cap;
        Mat previous_frame_float;

    public:
        int open_camera ();
        void get_current_frame (int flip_image = 0);
        char display_image (string title, Mat image);

    private:
        void check_for_previous_frame ();
};

#endif