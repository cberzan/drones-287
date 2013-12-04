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

    Mat frame;

    char const* imageNames[] = {
        "platform1.jpg",
        "platform2.jpg",
        "platform3.jpg",
        NULL
    };
    int imageIndex = 0;
    while(true) {
        frame = imread("../test/" + String(imageNames[imageIndex]));
        if (frame.empty())
        {
            cout << "Couldn't load input image" << endl;
            continue;
        }
        imageIndex++;
        if (!imageNames[imageIndex])
            imageIndex = 0;

        Mat_<double> corners = detectCorners(
            frame,
            inputWindowHandle,
            NULL, // cannyWindowHandle,
            contourWindowHandle);
        cout << corners << endl;
        Mat_<double> calibratedCorners = calibrateImagePoints(corners);
        cout << calibratedCorners << endl;
        if(corners.rows) {
            Mat_<double> simplePose = estimatePose(calibratedCorners);
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
