#include "engine.h"
#include "particle.h"

#include <iostream>
#include "opencv2/opencv.hpp"

#define _USE_MATH_DEFINES
#include <cmath>

using namespace std;
using namespace cv;


// helper functions
Mat convert_color_image_to_float(Mat image) {
    cvtColor(image, image, COLOR_BGR2GRAY);
    image.convertTo(image, CV_32FC1);
    return image;
}

Mat draw_color_bar (Mat image) {
    for (int r=0; r < int(image.rows / 20); r++) {
        for (int c=0; c < int(image.cols / 8); c++) {
            image.at<Vec3b>(r, c) = get_rgb_from_hsv((float)(int(c * 8 * 360 / image.cols)));
        }
    }
    return image;
}

Vec3b get_color(float x, float y) {
    float angle = atanf32(y / x) * 180 / M_PI;
    angle = map_atan_to_360_deg(x, y, angle); // maps arctan output to 360 degrees
    Vec3b color = get_rgb_from_hsv(angle);
    return color;
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

Mat resize_image (Mat image, float scale) {
    resize(image, image, Size(image.cols / scale, image.rows / scale));
    return image;
}


// module functions
void Engine::check_for_previous_frame () {
    if (previous_frame_float.empty()) {
        previous_frame_float = current_frame_float.clone();
    }
}

void Engine::compute_lk_flow () {
    for (int r = LK_WINDOW_DIM; r < (current_frame_float.rows - LK_WINDOW_DIM); r++) {
        for (int c = LK_WINDOW_DIM; c < (current_frame_float.cols - LK_WINDOW_DIM); c++) {
            
            Mat Ax = get_gradient_roi_vector(r, c, LK_WINDOW_DIM, x_gradient);
            Mat Ay = get_gradient_roi_vector(r, c, LK_WINDOW_DIM, y_gradient);
            Mat b = get_gradient_roi_vector(r, c, LK_WINDOW_DIM, t_gradient);

            Mat A;
            hconcat(Ax, Ay, A);

            Mat nu = (A.t() * A).inv() * A.t() * b; // compute flow vector
            nu = -10 * nu; // make negative to flip flow direction

            x_flow.at<float>(r, c) = nu.at<float>(0, 0);
            y_flow.at<float>(r, c) = nu.at<float>(1, 0);
        }
    }
}

void Engine::compute_t_gradient () {
    subtract(current_frame_float, previous_frame_float, t_gradient);
}

void Engine::compute_x_gradient () {
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

void Engine::destroy_all_windows () {
    destroyAllWindows();
}

char Engine::display_image (string title, Mat image, float resize_scale=0) {

    if (resize_scale != 0) {
        image = resize_image(image, (1 / resize_scale));
    }
    imshow(title, image);

    char key_press = waitKey(1);
    return key_press;
}

void Engine::draw_particles (Particle particles[]) {
    for (int i = 0; i < NUM_OF_BUBBLES; i++) {
        // x is columns, y is rows
        circle(current_frame_color, Point(int(particles[i].pos.x), int(particles[i].pos.y)), 3, particles[i].color, 2);
    }
}

void Engine::initialize_kernels () {
    x_kernel = Mat(1, 3, CV_32F);
    x_kernel.at<float>(0, 0) = -1.0f;
    x_kernel.at<float>(0, 1) = 0.0f;
    x_kernel.at<float>(0, 2) = 1.0f;

    y_kernel = Mat(3, 1, CV_32F);
    y_kernel.at<float>(0, 0) = -1.0f;
    y_kernel.at<float>(1, 0) = 0.0f;
    y_kernel.at<float>(2, 0) = 1.0f;
}

void Engine::initialize_lk_arrays () {
    x_flow = Mat::ones(current_frame_float.rows, current_frame_float.cols, CV_32FC1);
    y_flow = Mat::ones(current_frame_float.rows, current_frame_float.cols, CV_32FC1);
}

void Engine::get_current_frame () {
    cap >> current_frame_color;
    
    if (FLIP_IMAGE) {
        flip(current_frame_color, current_frame_color, 1);
    }
    current_frame_placeholder = convert_color_image_to_float(current_frame_color);
    current_frame_placeholder = resize_image(current_frame_placeholder, DOWNSAMPLE_SCALE);

    int top = LK_WINDOW_DIM, bottom = 50, left = 50, right = 50;

    copyMakeBorder(
        current_frame_placeholder, 
        current_frame_float, 
        LK_WINDOW_DIM, 
        LK_WINDOW_DIM, 
        LK_WINDOW_DIM, 
        LK_WINDOW_DIM, 
        BORDER_CONSTANT, 
        Scalar(0, 0, 0)
    );

    check_for_previous_frame();
}

Mat Engine::get_gradient_roi_vector (int r, int c, int window_dim, Mat gradient) {
    Mat roi = gradient(
        Range(r - window_dim, r + window_dim),
        Range(c - window_dim, c + window_dim)
    ).clone();

    roi = roi.reshape(1, roi.rows * roi.cols);
    return roi;
}

void Engine::move_particles (
    Particle particles[], 
    unordered_map<int, vector<int>>& particle_hash,
    FlowField* p
    ) {

    p->perlin_z += NOISE_Z_DELTA; // advance perlin noise field
    for (auto const& pair : particle_hash) {

        int key = pair.first;
        // coordinates in downsampled space
        int x = key % (current_frame_float.cols - LK_WINDOW_DIM*2);
        int y = (key - x) / (current_frame_float.cols - LK_WINDOW_DIM*2);

        // get noise
        double noise_angle = p->perlin.octave3D_01(
                (x * NOISE_X_SCALAR), 
                (y * NOISE_Y_SCALAR), 
                p->perlin_z, 
                4
            ) * M_PI * 4;
        float flow_x = cos(noise_angle);
        float flow_y = sin(noise_angle);

        // calculate lk flow
        Mat Ax = get_gradient_roi_vector(
            y + LK_WINDOW_DIM, 
            x + LK_WINDOW_DIM, 
            LK_WINDOW_DIM, 
            x_gradient
        );
        Mat Ay = get_gradient_roi_vector(
            y + LK_WINDOW_DIM, 
            x + LK_WINDOW_DIM, 
            LK_WINDOW_DIM, 
            y_gradient
        );
        Mat b = get_gradient_roi_vector(
            y + LK_WINDOW_DIM, 
            x + LK_WINDOW_DIM, 
            LK_WINDOW_DIM, 
            t_gradient
        );

        Mat A;
        hconcat(Ax, Ay, A);

        Mat nu = (A.t() * A).inv() * A.t() * b; // compute flow vector
        nu = -10 * nu; // make negative to flip flow direction

        float add_x = nu.at<float>(0, 0);
        float add_y = nu.at<float>(1, 0);

        // iterate through particles
        for (auto const& i : pair.second) {
            // flow threshold reached, color with direction and add acceleration
            if (sqrtf32(pow(add_x, 2) + pow(add_y, 2)) > FLOW_THRESHOLD) {
                float angle = atanf32(add_y / add_x) * 180 / M_PI;
                angle = map_atan_to_360_deg(add_x, add_y, angle); // maps arctan output to 360 degrees
                add(&particles[i].acc, (add_x * ACC_SCALE), (add_y * ACC_SCALE));
                particles[i].color = get_rgb_from_hsv(angle);
            } 
            // threshold not reached, particle remains white
            else {
                Vec3b color;
                color[0] = 255;
                color[1] = 255;
                color[2] = 255;
                particles[i].color = color;
            }

            // add optical flow acceleration, user "pushing" particles
            add(&particles[i].acc, (flow_x * FLOW_SCALE), (flow_y * FLOW_SCALE));
            // update physics
            update_vec(&particles[i].vel, &particles[i].acc);
            update_vec(&particles[i].pos, &particles[i].vel);
            // keep particles in window
            check_window_bound(&particles[i].pos, current_frame_color.cols, current_frame_color.rows);
            // keep particles within speed range
            check_magnitude_limit(&particles[i].vel);
            check_magnitude_limit(&particles[i].acc);
            // dampen the motion of particles after push
            dampen(&particles[i].vel, VEL_DAMPEN_COEFF);
            dampen(&particles[i].acc, ACC_DAMPEN_COEFF);
        }
    }

    particle_hash.clear();
    for (int i = 0; i < NUM_OF_BUBBLES; i++) {
        // linear index for each point
        int key = floor(particles[i].pos.y / DOWNSAMPLE_SCALE)
            * floor(current_frame_color.cols / DOWNSAMPLE_SCALE) 
            + floor(particles[i].pos.x / DOWNSAMPLE_SCALE);

        if (particle_hash.find(key) == particle_hash.end()) {
            // not found
            particle_hash[key] = {i};
        } else {
            particle_hash[key].push_back(i);
        }
    }
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

void Engine::release_cap () {
    cap.release();
}

void Engine::store_previous_frame () {
    previous_frame_float = current_frame_float.clone();
}

// to view how particles are grouped from downsample
void Engine::visualize_downsample () {
    // rows
    for (float r=1; r < current_frame_float.rows; r++) {
        int row_line = floor(r / current_frame_float.rows * current_frame_color.rows);
        line(
            current_frame_color, 
            Point(0, row_line), 
            Point(current_frame_color.cols, row_line), 
            Scalar(0, 0, 100), 
            2
        );
    }
    // columns
    for (float c=1; c < current_frame_float.cols; c++) {
        int col_line = floor(c / current_frame_float.cols * current_frame_color.cols);
        line(
            current_frame_color, 
            Point(col_line, 0), 
            Point(col_line, current_frame_color.rows), 
            Scalar(0, 0, 100), 
            2
        );
    }
}

void Engine::visualize_lk_flow () {
    // reset background
    flow = Mat::zeros(current_frame_float.rows, current_frame_float.cols, CV_8UC3);
    int flow_threshold { 10 };
    for (int r = 0; r < current_frame_float.rows; r++) {
        for (int c = 0; c < current_frame_float.cols; c++) {
            float x = x_flow.at<float>(r, c);
            float y = y_flow.at<float>(r, c);
            if (sqrtf32(pow(x, 2) + pow(y, 2)) > flow_threshold) {
                Vec3b color = get_color(x, y);
                flow.at<Vec3b>(r, c) = color;
            }
        }
    }
    flow = draw_color_bar(flow);
}