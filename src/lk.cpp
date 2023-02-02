#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "../include/particle.h"
#include <random>

#define _USE_MATH_DEFINES
#include <cmath>


using namespace std;
using namespace cv;


float scale { 10 };
float flow_threshold { 5 };
int window_dim { 10 };



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


float map_atan_to_360_deg(float x, float y, float angle) {
    if (x > 0) {
        if (angle < 0) {
            angle += 360.0f;
        }
    } else {
        angle += 180.0f;
    }
    return angle;
}

Vec3b get_rgb_from_hsv(float angle) {

    Vec3b color;
    
    float H = angle / 60.0f;
    float V = 1.0f;
    float S = 1.0f;

    float C = V * S;
    int X_rgb = (int)(C * (1 - fabs(fmod(H, 2.0f) - 1)) * 255.0f);
    int C_rgb = (int)(C * 255.0f);

    if (H >= 0 & H < 1) {
        color[0] = C_rgb;
        color[1] = X_rgb;
        color[2] = 0;
    } else if (H >= 1 & H < 2) {
        color[0] = X_rgb;
        color[1] = C_rgb;
        color[2] = 0;
    } else if (H >= 2 & H < 3) {
        color[0] = 0;
        color[1] = C_rgb;
        color[2] = X_rgb;
    } else if (H >= 3 & H < 4) {
        color[0] = 0;
        color[1] = X_rgb;
        color[2] = C_rgb;
    } else if (H >= 4 & H < 5) {
        color[0] = X_rgb;
        color[1] = 0;
        color[2] = C_rgb;
    } else if (H >= 5 & H < 6) {
        color[0] = C_rgb;
        color[1] = 0;
        color[2] = X_rgb;
    }
    

    return color;

}

Mat draw_color_bar (Mat image) {
    for (int r=0; r < int(image.rows / 20); r++) {
        for (int c=0; c < int(image.cols / 8); c++) {
            image.at<Vec3b>(r, c) = get_rgb_from_hsv((float)(int(c * 8 * 360 / image.cols)));
        }
    }
    return image;
}


int main () {

    int num {1000};
    Particle bubs[num];

    random_device rd; // obtain a random number from hardware
    mt19937 gen(rd()); // seed the generator
    uniform_int_distribution<> distr_pos(100, 900); // define the range
    uniform_int_distribution<> distr_vec(-10, 10); // define the range


    for (int i = 0; i < num; i++) {
        bubs[i].initialize_vectors(distr_pos(gen), distr_pos(gen), 0, 0, 0, 0);
        // bubs[i].initialize_vectors(distr_pos(gen), distr_pos(gen), distr_vec(gen), distr_vec(gen), 0, 0);
        bubs[i].vel.magnitude_limit = 50;
        bubs[i].acc.magnitude_limit = 50;

        bubs[i].vel.dampening_coeff = 0.15;
        bubs[i].acc.dampening_coeff = 0.15;
    }


    string windows[] = {
        "X Gradient",
        "Y Gradient",
        "T Gradient",
        "Frame",
        "LK X",
        "LK Y",
        "flow"
    };
    initialize_windows(windows);

    // initialize camera capture
    cout << "Opening camera..." << endl; 
    VideoCapture cap(0);
    if(!cap.isOpened()) {
        cout << "Camera open failed" << endl;
        return 1;
    }

    Mat previous_frame;
    Mat previous_frame_scaled;


    while (1) {
        
        Mat current_frame;
        Mat current_frame_color;

        cap >> current_frame_color;
        cvtColor(current_frame_color, current_frame, COLOR_BGR2GRAY);
        flip(current_frame, current_frame, 1);
        flip(current_frame_color, current_frame_color, 1);


        if (current_frame.empty())
            break;
        if (previous_frame.empty()) {
            previous_frame = current_frame.clone();
            previous_frame_scaled = resize_image(previous_frame, scale);
            previous_frame_scaled.convertTo(previous_frame_scaled, CV_32FC1);
        }

        Mat current_frame_scaled = resize_image(current_frame, scale);
        current_frame_scaled.convertTo(current_frame_scaled, CV_32FC1);


        // compute gradient matrices 
        Mat t_gradient;
        Mat x_gradient;
        Mat y_gradient;

        Mat x_kernel(1, 3, CV_32F);
        x_kernel.at<float>(0, 0) = -1.0f;
        x_kernel.at<float>(0, 1) = 0.0f;
        x_kernel.at<float>(0, 2) = 1.0f;

        Mat y_kernel(3, 1, CV_32F);
        y_kernel.at<float>(0, 0) = -1.0f;
        y_kernel.at<float>(1, 0) = 0.0f;
        y_kernel.at<float>(2, 0) = 1.0f;

        subtract(current_frame_scaled, previous_frame_scaled, t_gradient);
        filter2D(current_frame_scaled, x_gradient, -1 , x_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);
        filter2D(current_frame_scaled, y_gradient, -1 , y_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);


        // initialize flow matrices
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
                nu = -10 * nu; // make negative to flip flow direction

                u.at<float>(r, c) = nu.at<float>(0, 0);
                v.at<float>(r, c) = nu.at<float>(1, 0);

            }
        }




        // visualize LK flow, color = direction
        Mat flow = Mat::zeros(current_frame_scaled.rows, current_frame_scaled.cols, CV_8UC3);
        for (int r=0; r < current_frame_scaled.rows; r++) {
            for (int c=0; c < current_frame_scaled.cols; c++) {
                float x = u.at<float>(r, c);
                float y = v.at<float>(r, c);


                if (sqrtf32(pow(x, 2) + pow(y, 2)) > flow_threshold) {

                    float angle = atanf32(y / x) * 180 / M_PI;
                    angle = map_atan_to_360_deg(x, y, angle); // maps arctan output to 360 degrees
                    Vec3b color = get_rgb_from_hsv(angle);

                    flow.at<Vec3b>(r, c) = color;
                }
            }
        }


        flow = resize_image(flow, 1 / scale);
        flow = draw_color_bar(flow);

        for (int i = 0; i < num; i++) {
            int padding {130};
            float timestep { 0.6 };


            bubs[i].update(timestep, flow.cols, flow.rows, padding);
            circle(current_frame_color, Point(int(bubs[i].pos.x), int(bubs[i].pos.y)), 5, Scalar(255, 50, 50), 2);
    

            float add_x = u.at<float>(int(bubs[i].pos.y / scale), int(bubs[i].pos.x / scale));
            float add_y = v.at<float>(int(bubs[i].pos.y / scale), int(bubs[i].pos.x / scale));

            bubs[i].acc.add(add_x * 1, add_y * 1); 

            bubs[i].vel.dampening();
            bubs[i].acc.dampening();

            // bubs[i].acc.add(bubs[i].acc.x * -0.15, bubs[i].acc.y * -0.15);



        }

        imshow("flow", flow);
        imshow("Frame", current_frame_color);


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