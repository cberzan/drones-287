#include <iostream>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

int main(int argc, char *argv[])
{
    if(argc != 2) {
        cerr << "Usage: " << argv[0] << " out.jpg" << endl;
        return 1;
    }

    VideoCapture cap(0);
    if(!cap.isOpened()) {
        cerr << "Could not open camera." << endl;
        return 1;
    }

    Mat frame;
    cap >> frame;
    imwrite(argv[1], frame);
}
