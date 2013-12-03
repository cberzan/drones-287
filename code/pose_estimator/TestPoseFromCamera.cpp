#include "Corners.h"
#include "Geometry.h"

#include <iostream>
using namespace std;

#define ESCAPE_KEY 27


int main()
{
    char const* inputWindowHandle = "Input";
    char const* cannyWindowHandle = "Canny";
    char const* contourWindowHandle = "Contours";
    namedWindow(inputWindowHandle, CV_WINDOW_AUTOSIZE);
    // namedWindow(cannyWindowHandle, CV_WINDOW_AUTOSIZE);
    namedWindow(contourWindowHandle, CV_WINDOW_AUTOSIZE);

    VideoCapture capture(0);
    if(!capture.isOpened()) {
        cerr << "Device inaccessible. Damn!" << endl;
        return -1;
    }

    Mat frame;
    while(true) {
        capture >> frame;
        Mat_<double> corners = detectCorners(
            frame,
            inputWindowHandle,
            NULL, // cannyWindowHandle,
            contourWindowHandle);
        if(corners.rows) {
            Mat_<double> simplePose = estimatePose(corners);
            cout << simplePose << endl;
        } else {
            cout << "could not detect all corners" << endl;
        }

        if ((char)waitKey(0) == ESCAPE_KEY)
            break;
    }
    destroyAllWindows();
    return 0;
}
