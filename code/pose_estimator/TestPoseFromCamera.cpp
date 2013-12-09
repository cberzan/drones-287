#include "Corners.h"
#include "Geometry.h"

#include <iostream>
using namespace std;

#define ESCAPE_KEY 27


int main(int argc, char *argv[])
{
    bool record = false;
    if(argc == 2 && strcmp(argv[1], "--record") == 0) {
        record = true;
    }

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

    // Set exposure and focus manually.
    // Seems to take effect after several frames.
    capture >> frame;
    system("v4l2-ctl -d 0 -c exposure_auto=1");
    system("v4l2-ctl -d 0 -c exposure_absolute=180");
    system("v4l2-ctl -d 0 -c focus_auto=0");
    system("v4l2-ctl -d 0 -c focus_absolute=0");

    int frameNo = 0;
    while(true) {
        capture >> frame;
        Mat_<double> corners = detectCorners(
            frame,
            inputWindowHandle,
            NULL, // cannyWindowHandle,
            contourWindowHandle);
        if(corners.rows) {
            Mat_<double> calibratedCorners = calibrateImagePoints(corners);
            Mat_<double> simplePose = estimatePose(calibratedCorners);
            cout << simplePose << endl;
        } else {
            cout << "could not detect all corners" << endl;
        }

        if(record) {
            stringstream s;
            s.width(10);
            s.fill('0');
            s << frameNo << ".jpg";
            imwrite(s.str(), frame);
            cout << "wrote " << s.str() << endl;
            frameNo++;
        }

        if ((char)waitKey(100) == ESCAPE_KEY)
            break;
    }
    destroyAllWindows();
    return 0;
}
