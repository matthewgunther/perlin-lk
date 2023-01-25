#include <iostream>
#include <opencv4/opencv2/opencv.hpp>
#include "../include/particle.h"

#define _USE_MATH_DEFINES
#include <cmath>

// #include <cstdlib>


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

int main () {

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
        "flow"
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
            previous_frame_scaled.convertTo(previous_frame_scaled, CV_32FC1);
        }

        Mat current_frame_scaled = resize_image(current_frame, scale);
        current_frame_scaled.convertTo(current_frame_scaled, CV_32FC1);

        Mat t_gradient;
        subtract(current_frame_scaled, previous_frame_scaled, t_gradient);



        Mat x_gradient;
        Mat y_gradient;
        filter2D(current_frame_scaled, x_gradient, -1 , x_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);
        filter2D(current_frame_scaled, y_gradient, -1 , y_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);



        
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

                // cout << "Ax \n" << Ax << endl;
                // cout << "Ay \n" << Ay << endl;
                // cout << "b \n" << b << endl;


            }
        }

        

        

        // show_image("X Gradient", x_gradient);
        // show_image("Y Gradient", y_gradient);
        // show_image("T Gradient", t_gradient);



        // int window_width { 700 };
        // int window_height { 500 };
        // Mat frame_bub = Mat::zeros(window_height, window_width, CV_8UC3);

        // int window_width = current_frame.cols;
        // int window_height = current_frame.rows;
        // Mat frame_bub = Mat::zeros(window_height, window_width, CV_8UC3);
        // circle(frame_bub, Point(int(bub.pos_x), int(bub.pos_y)), 20, Scalar(0, 255, 0), -1);
        // imshow("bub", frame_bub);

        // bub.vel_x = bub.vel_x + u.at<float>(int(bub.pos_x / scale), int(bub.pos_y / scale)) * 10;
        // // bub.vel_y = u.at<float>(bub.pos_x, bub.pos_y) * 5;
        // bub.update(0.2, window_width, window_height);
        // cout << bub.vel_x  << " - " << bub.vel_y <<  endl;




        // u.convertTo(u, CV_8UC1);
        // v.convertTo(v, CV_8UC1);

        Mat flow = Mat::zeros(current_frame_scaled.rows, current_frame_scaled.cols, CV_8UC3);
        for (int r=0; r < current_frame_scaled.rows; r++) {
            for (int c=0; c < current_frame_scaled.cols; c++) {
                float x = u.at<float>(r, c);
                float y = v.at<float>(r, c);


                if (sqrtf32(pow(x, 2) + pow(y, 2)) > flow_threshold) {

                

                    float angle = atanf32(y / x) * 180 / M_PI;
                    angle = map_atan_to_360_deg(x, y, angle); // maps arctan output to 360 degrees
                    Vec3b color = get_rgb_from_hsv(angle);

                    // cout << x << " / " << y << " / " << angle << " / " << color << endl;

                    flow.at<Vec3b>(r, c) = color;
                }
            }
        }

        // cout << "0" << " - " << get_rgb_from_hsv(0) << endl;
        // cout << "90" << " - " << get_rgb_from_hsv(90) << endl;
        // cout << "180" << " - " << get_rgb_from_hsv(180) << endl;
        // cout << "360" << " - " << get_rgb_from_hsv(270) << endl;
        // cout << v.rows << " - " << v.cols << endl;

        for (int r=0; r<3; r++) {
            for (int c=0; c<36; c++) {
                flow.at<Vec3b>(r, c) = get_rgb_from_hsv((float)(c * 10));
            }
        }


        flow = resize_image(flow, 1/scale);
        flow = resize_image(flow, 1.2);
        current_frame = resize_image(current_frame, 1.2);
        imshow("flow", flow);
        imshow("Frame", current_frame);



        // // flow = resize_image(flow, 1 / scale);
        // // current_frame.convertTo(current_frame, CV_8UC3);

        // // cout << flow.rows  << " - " << flow.cols <<  endl;
        // // cout << current_frame.rows  << " - " << current_frame.cols <<  endl;

        // // Mat sub;
        // // subtract(current_frame, flow, sub);

        // Mat foreground = flow;
        // Mat background = current_frame_scaled;
        // Mat alpha = flow;

        // // Convert Mat to float data type
        // foreground.convertTo(foreground, CV_32FC3);
        // background.convertTo(background, CV_32FC3);

        // // Normalize the alpha mask to keep intensity between 0 and 1
        // alpha.convertTo(alpha, CV_32FC3, 1.0/255); // 

        // // Storage for output image
        // Mat ouImage = Mat::zeros(foreground.size(), foreground.type());

        // // Multiply the foreground with the alpha matte
        // multiply(alpha, foreground, foreground); 
        // // Multiply the background with ( 1 - alpha )
        // multiply(Scalar::all(1.0)-alpha, background, background); 

        // // Add the masked foreground and background.
        // add(foreground, background, ouImage); 

        // // Display image
        // imshow("alpha blended image", ouImage/255);

        
        // // show_image("flow", flow);
 


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