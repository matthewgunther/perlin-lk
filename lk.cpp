#include <iostream>
#include <opencv4/opencv2/opencv.hpp>

using namespace std;
using namespace cv;


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


    float scale = 4;

    while (1) {
        Mat frame;

        cap >> frame;

        resize(frame, frame, Size(frame.cols / scale, frame.rows / scale));


        if (frame.empty())
            break;

        Mat x_gradient;
        filter2D(frame, x_gradient, -1 , x_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);

        Mat y_gradient;
        filter2D(frame, y_gradient, -1 , y_kernel, Point( -1, -1 ), 0, BORDER_DEFAULT);
        
        // imshow("Video capture", frame);
        // imshow("X Gradient", x_gradient);

        resize(y_gradient, y_gradient, Size(y_gradient.cols * scale, y_gradient.rows * scale));

        imshow("Y Gradient", y_gradient);

        char key_press = waitKey(10);
        if (key_press==27)
            break;
    }
    
    cap.release();
    destroyAllWindows();
    return 0;
}