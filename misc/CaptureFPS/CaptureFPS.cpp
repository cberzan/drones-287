#include <cstdio>
#include <cstdlib>
#include <signal.h>
#include <sys/time.h>
#include <opencv2/opencv.hpp>
using namespace cv;
using namespace std;

static bool keepRunning = true;

void intHandler(int dummy = 0)
{
    keepRunning = false;
}

double get_wall_time()
{
    struct timeval time;
    gettimeofday(&time, NULL);  // FIXME: ignoring errors
    return (double)time.tv_sec + (double)time.tv_usec * .000001;
}

int main(int argc, char *argv[])
{
    signal(SIGINT, intHandler);

    bool display = false;
    if(argc == 2 && strcmp(argv[1], "--display") == 0) {
        display = true;
    }

    VideoCapture cap(0);
    if(!cap.isOpened()) {
        fprintf(stderr, "Could not open camera.\n");
        return -1;
    }
    printf("Camera properties:\n");
    printf("Frame width:  %f\n", cap.get(CV_CAP_PROP_FRAME_WIDTH));
    printf("Frame height: %f\n", cap.get(CV_CAP_PROP_FRAME_HEIGHT));
    printf("FPS:          %f\n", cap.get(CV_CAP_PROP_FPS));

    // Getting or setting FPS doesn't seem to work.
    // See http://answers.opencv.org/question/6713/#post-id-6869
    // cap.set(CV_CAP_PROP_FPS, 60);
    // printf("FPS:          %f\n", cap.get(CV_CAP_PROP_FPS));

    double start = get_wall_time();
    int frameNo = 0;
    while(keepRunning) {
        Mat frame;
        cap >> frame;
        stringstream s;
        s.width(10);
        s.fill('0');
        s << frameNo << ".jpg";
        imwrite(s.str(), frame);
        cout << "wrote " << s.str() << endl;
        if(display) {
            imshow("CaptureFPS", frame);
            // Give highgui a chance to event-loop.
            // (Otherwise no image shows up. Go figure.)
            waitKey(1);
        }
        frameNo++;
	}
    printf("\n");
    double elapsed = get_wall_time() - start;
    printf("Captured %d frames in %f seconds (%f FPS).\n",
        frameNo, elapsed, frameNo / elapsed);
	return 0; 
}
