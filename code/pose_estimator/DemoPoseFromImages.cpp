#include "Corners.h"
#include "Geometry.h"

#include <cstdio>
#include <iostream>
using namespace std;

#define ESCAPE_KEY 27


int main(int argc, char **argv)
{
    char const* inputWindowHandle = "Input";
    char const* cannyWindowHandle = "Canny";
    char const* contourWindowHandle = "Contours";
    namedWindow(inputWindowHandle, CV_WINDOW_AUTOSIZE);
    // namedWindow(cannyWindowHandle, CV_WINDOW_AUTOSIZE);
    namedWindow(contourWindowHandle, CV_WINDOW_AUTOSIZE);

    // Draw something on the windows, so that they are the right size.
    Mat dummy = Mat::zeros(480, 640, CV_8UC3);
    imshow(inputWindowHandle, dummy);
    imshow(contourWindowHandle, dummy);

    cout << "Press any key to start..." << endl;
    waitKey(0);

    Mat frame;
    for(int i = 1; i < argc; i++) {
        frame = imread(argv[i]);
        if (frame.empty()) {
            cout << "Couldn't load input image" << endl;
            continue;
        }

        Mat_<double> corners = detectCorners(
            frame,
            inputWindowHandle,
            NULL, // cannyWindowHandle,
            contourWindowHandle);
        if(corners.rows) {
            Mat_<double> calibratedCorners = calibrateImagePoints(corners);
            Mat_<double> simplePose = estimatePose(calibratedCorners);
            // cout << corners << endl;
            // cout << calibratedCorners << endl;
            // cout << simplePose << endl;
            printf(
                "x=% 6.2f   y=% 6.2f   z=% 6.2f   yaw=% 6.2f deg\n",
                simplePose(0),
                simplePose(1),
                simplePose(2),
                simplePose(3) * 180 / M_PI);
        } else {
            cout << "could not detect all corners" << endl;
        }

        // Assumes images were captured at 30 FPS.
        if ((char)waitKey(33) == ESCAPE_KEY) {
            break;
        }
    }
    cout << "Press any key to exit..." << endl;
    waitKey(0);
    destroyAllWindows();
    return 0;
}
