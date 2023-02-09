#include "engine.h"
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Mat convert_color_image_to_float(Mat image) {
    cvtColor(image, image, COLOR_BGR2GRAY);
    image.convertTo(image, CV_32FC1);
    return image;
}

int Engine::open_camera () {
    cout << "Opening camera..."; 
    VideoCapture cap_init(0);
    if (!cap_init.isOpened()) {
        cout << "FAILED camera open" << endl;
        return 1;
    }
    cout << "SUCCESSFUL camera open" << endl; 
    cap = cap_init;
    return 0;
}

void Engine::check_for_previous_frame () {
    if (previous_frame_float.empty()) {
        previous_frame_float = convert_color_image_to_float(current_frame_color.clone());
    }
}

void Engine::get_current_frame (int flip_image) {
    cap >> current_frame_color;
    if (flip_image)
        flip(current_frame_color, current_frame_color, 1);
    current_frame_float = convert_color_image_to_float(current_frame_color);
    check_for_previous_frame();
}

char Engine::display_image (string title, Mat image) {
    imshow(title, image);
    char key_press = waitKey(0);

    if (key_press == 27)
        destroyAllWindows();
    return key_press;
}

void Engine::compute_t_gradient () {
    subtract(current_frame_float, previous_frame_float, t_gradient);
}

void Engine::compute_x_gradient () {
    Mat x_kernel(1, 3, CV_32F);
    x_kernel.at<float>(0, 0) = -1.0f;
    x_kernel.at<float>(0, 1) = 0.0f;
    x_kernel.at<float>(0, 2) = 1.0f;
    filter2D(
        current_frame_float,
        x_gradient, 
        -1 , 
        x_kernel, 
        Point(-1, -1), 
        0, 
        BORDER_DEFAULT
    );
    display_image("x", x_gradient);
}

void Engine::store_previous_frame () {
    previous_frame_float = current_frame_float.clone();
}



