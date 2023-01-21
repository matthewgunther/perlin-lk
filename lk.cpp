#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;


Mat resize_image (Mat input_image, float scale) {
    Mat output_image;
    resize(input_image, output_image, Size(input_image.cols / scale, input_image.rows / scale));
    return output_image;
}

template <size_t N>
void destroy_windows(string (&windows)[N]) {
    for (const string window_name : windows) {
        destroyWindow(window_name);
    }
}

template <size_t N>
void initialize_windows(string (&windows)[N]) {
    for (const string window_name : windows) {
        namedWindow(window_name);
    }
}


int main () {
    cout << "Opening camera..." << endl; 

    VideoCapture cap(0);

    if(!cap.isOpened()) {
        cout << "Camera open failed" << endl;
        return 1;
    }

    Mat x_kernel(1, 3, CV_32F);
    x_kernel.at<float>(0, 0) = -1.0f;
    x_kernel.at<float>(0, 1) = 0.0f;
    x_kernel.at<float>(0, 2) = 1.0f;

    Mat y_kernel(3, 1, CV_32F);
    y_kernel.at<float>(0, 0) = -1.0f;
    y_kernel.at<float>(1, 0) = 0.0f;
    y_kernel.at<float>(2, 0) = 1.0f;

    string windows[] = {
        "X Gradient",
        "Y Gradient",
        "T Gradient",
        "Frame",
    };

    initialize_windows(windows);

    float scale {10.0};

    Mat previous_frame;
    Mat previous_frame_scaled;

    while (1) {
        Mat current_frame;
        cap >> current_frame;

        if (current_frame.empty())
            break;
        if (previous_frame.empty()) {
            previous_frame = current_frame.clone();
            previous_frame_scaled = resize_image(previous_frame, scale);
        }

        Mat x_gradient;
        Mat y_gradient;

        Mat current_frame_scaled = resize_image(current_frame, scale);

        filter2D(current_frame_scaled, x_gradient, -1 , x_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);
        filter2D(current_frame_scaled, y_gradient, -1 , y_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);
        
        x_gradient = resize_image(x_gradient, 1 / scale);
        y_gradient = resize_image(y_gradient, 1 / scale);
        imshow("X Gradient", x_gradient);
        imshow("Y Gradient", y_gradient);

        Mat t_gradient;
        subtract(current_frame_scaled, previous_frame_scaled, t_gradient);
        t_gradient = resize_image(t_gradient, 1 / scale);
        imshow("T Gradient", t_gradient);


        char key_press = waitKey(10);
        if (key_press==27)
            break;
        previous_frame = current_frame.clone();
        previous_frame_scaled = current_frame_scaled.clone();
    }
    
    cap.release();
    destroy_windows(windows);
    return 0;
}