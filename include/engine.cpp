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

Mat resize_image (Mat image, float scale) {
    resize(image, image, Size(image.cols / scale, image.rows / scale));
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
        previous_frame_float = current_frame_float.clone();
    }
}

void Engine::get_current_frame (int flip_image, float downsample_scale) {
    cap >> current_frame_color;
    if (flip_image)
        flip(current_frame_color, current_frame_color, 1);
    current_frame_float = convert_color_image_to_float(current_frame_color);
    current_frame_float = resize_image(current_frame_float, downsample_scale);
    check_for_previous_frame();
}

char Engine::display_image (string title, Mat image) {
    imshow(title, image);
    char key_press = waitKey(1);

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
}

void Engine::compute_y_gradient () {
    Mat y_kernel(3, 1, CV_32F);
    y_kernel.at<float>(0, 0) = -1.0f;
    y_kernel.at<float>(1, 0) = 0.0f;
    y_kernel.at<float>(2, 0) = 1.0f;
    filter2D(
        current_frame_float,
        y_gradient, 
        -1 , 
        y_kernel, 
        Point(-1, -1), 
        0, 
        BORDER_DEFAULT
    );
}

Mat Engine::get_gradient_roi_vector (int r, int c, int window_dim, Mat gradient) {
    Mat roi = gradient(
        Range(r - window_dim, r + window_dim + 1),
        Range(c - window_dim, c + window_dim + 1)
    ).clone();
    roi = roi.reshape(1, roi.rows * roi.cols);
    return roi;
}

void Engine::compute_lk_flow (int window_dim) {
    // initialize flow matrices
    x_flow = Mat::ones(current_frame_float.rows, current_frame_float.cols, CV_32FC1);
    y_flow = Mat::ones(current_frame_float.rows, current_frame_float.cols, CV_32FC1);

    for (int r = window_dim; r < (current_frame_float.rows - window_dim); r++) {
        for (int c = window_dim; c < (current_frame_float.cols - window_dim); c++) {
            
            Mat Ax = get_gradient_roi_vector(r, c, window_dim, x_gradient);
            Mat Ay = get_gradient_roi_vector(r, c, window_dim, y_gradient);
            Mat b = get_gradient_roi_vector(r, c, window_dim, t_gradient);

            Mat A;
            hconcat(Ax, Ay, A);

            Mat nu = (A.t() * A).inv() * A.t() * b; // compute flow vector
            nu = -10 * nu; // make negative to flip flow direction

            x_flow.at<float>(r, c) = nu.at<float>(0, 0);
            y_flow.at<float>(r, c) = nu.at<float>(1, 0);
        }
    }
}

void Engine::store_previous_frame () {
    previous_frame_float = current_frame_float.clone();
}



