#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "../include/particle.h"

using namespace std;
using namespace cv;


float scale { 4 };
int window_dim { 4 };


Mat resize_image (Mat input_image, float scale) {
    Mat output_image;
    resize(input_image, output_image, Size(input_image.cols / scale, input_image.rows / scale));
    return output_image;
}

void show_image(string window_name, Mat image) {
    image = resize_image(image, 1 / scale); 
    imshow(window_name, image);
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

    // Mat fr = Mat::zeros(100, 200, CV_8UC3);

    Particle bub = Particle(
        5, 10,
        10, 10,
        0.5, 0
    );



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
        "LK X",
        "LK Y",
        "bub"
    };

    initialize_windows(windows);


    Mat previous_frame;
    Mat previous_frame_scaled;

    while (1) {
        

        




        Mat current_frame;
        cap >> current_frame;
        cvtColor(current_frame, current_frame, COLOR_BGR2GRAY);




        
        

        if (current_frame.empty())
            break;
        if (previous_frame.empty()) {
            previous_frame = current_frame.clone();
            previous_frame_scaled = resize_image(previous_frame, scale);
        }

        Mat current_frame_scaled = resize_image(current_frame, scale);

        Mat x_gradient;
        Mat y_gradient;
        filter2D(current_frame_scaled, x_gradient, -1 , x_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);
        filter2D(current_frame_scaled, y_gradient, -1 , y_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);

        Mat t_gradient;
        subtract(current_frame_scaled, previous_frame_scaled, t_gradient);


        
        Mat u = Mat::ones(current_frame_scaled.rows, current_frame_scaled.cols, CV_32FC1);
        Mat v = Mat::ones(current_frame_scaled.rows, current_frame_scaled.cols, CV_32FC1);

        for (int r = window_dim; r < (current_frame_scaled.rows - window_dim); r++) {
            for (int c = window_dim; c < (current_frame_scaled.cols - window_dim); c++) {

                Mat Ax = x_gradient(
                    Range(r - window_dim, r + window_dim + 1),
                    Range(c - window_dim, c + window_dim + 1)
                ).clone();
                Mat Ay = y_gradient(
                    Range(r - window_dim, r + window_dim + 1),
                    Range(c - window_dim, c + window_dim + 1)
                ).clone();
                Mat b = t_gradient(
                    Range(r - window_dim, r + window_dim + 1),
                    Range(c - window_dim, c + window_dim + 1)
                ).clone();

                Ax = Ax.reshape(1, Ax.rows * Ax.cols);
                Ay = Ay.reshape(1, Ay.rows * Ay.cols);
                b = b.reshape(1, b.rows * b.cols);

                Mat A;
                hconcat(Ax, Ay, A);

                A.convertTo(A, CV_32FC1);
                b.convertTo(b, CV_32FC1);

                Mat nu = (A.t() * A).inv() * A.t() * b; // compute flow vector
                nu = nu * 10;

                u.at<float>(r, c) = nu.at<float>(0, 0);
                v.at<float>(r, c) = nu.at<float>(1, 0);
            }
        }

        

        

        // show_image("X Gradient", x_gradient);
        // show_image("Y Gradient", y_gradient);
        // show_image("T Gradient", t_gradient);



        // int window_width { 700 };
        // int window_height { 500 };
        // Mat frame_bub = Mat::zeros(window_height, window_width, CV_8UC3);

        int window_width = current_frame.cols;
        int window_height = current_frame.rows;
        Mat frame_bub = Mat::zeros(window_height, window_width, CV_8UC3);
        circle(frame_bub, Point(int(bub.pos_x), int(bub.pos_y)), 20, Scalar(0, 255, 0), -1);
        imshow("bub", frame_bub);

        bub.vel_x = bub.vel_x + u.at<float>(int(bub.pos_x / scale), int(bub.pos_y / scale)) * 10;
        // bub.vel_y = u.at<float>(bub.pos_x, bub.pos_y) * 5;
        bub.update(0.2, window_width, window_height);
        cout << bub.vel_x  << " - " << bub.vel_y <<  endl;




        u.convertTo(u, CV_8UC1);
        v.convertTo(v, CV_8UC1);
        show_image("LK X", u);
        show_image("LK Y", v);



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