#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <string>
#include <sys/time.h>
#include <time.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

static bool keepRunning = true;

void intHandler(int dummy = 0)
{
    keepRunning = false;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, intHandler);

    VideoCapture cap(0);
    if(!cap.isOpened()) {
        fprintf(stderr, "Could not open camera.\n");
        return -1;
    }

    // Set exposure manually.
    // Need to capture something first, or else it's unreliable.
    Mat dummy;
    cap >> dummy;
    system("v4l2-ctl -d /dev/video0 -c exposure_auto=1");
    system("v4l2-ctl -d /dev/video0 -c exposure_absolute=666");

    for(int exposure = 100; keepRunning && exposure < 2001; exposure += 100) {
        // Set exposure.
        char command[100];
        snprintf(
            command,
            100,
            "v4l2-ctl -d /dev/video0 -c exposure_absolute=%d",
            exposure);
        system(command);

        // Capture and save frame.
        // HACK: The exposure change isn't immediate, so capture
        // a number of frames instead. Save only the last one.
        Mat frame;
        for(int i = 0; i < 10; i++) {
            cap >> frame;
            // system(command);
        }
        char filename[100];
        snprintf(filename, 100, "%04d.jpeg", exposure);
        imwrite(filename, frame);
        cout << "wrote " << filename << endl;
	}
    cout << "Wrote images to current dir." << endl;
}
