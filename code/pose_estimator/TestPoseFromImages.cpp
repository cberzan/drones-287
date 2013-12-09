#include "Corners.h"
#include "Geometry.h"

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

    Mat frame;

    int imageIndex = 1;
    while(true) {
        frame = imread(argv[imageIndex]);
        if (frame.empty())
        {
            cout << "Couldn't load input image" << endl;
            continue;
        }
        imageIndex++;
		if (imageIndex == argc)
			imageIndex = 1;

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
