#include "engine.h"
#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Mat convert_image_to_bw(Mat image) {
    cvtColor(image, image, COLOR_BGR2GRAY);
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
        previous_frame_float = convert_image_to_bw(current_frame_color.clone());
        previous_frame_float.convertTo(previous_frame_float, CV_32FC1);
    }
}


void Engine::get_current_frame (int flip_image) {
    cap >> current_frame_color;
    if (flip_image)
        flip(current_frame_color, current_frame_color, 1);
    current_frame_bw = convert_image_to_bw(current_frame_color);
    check_for_previous_frame();
}

char Engine::display_image (string title, Mat image) {
    imshow(title, image);
    char key_press = waitKey(0);

    if (key_press == 27)
        destroyAllWindows();
    return key_press;
}

