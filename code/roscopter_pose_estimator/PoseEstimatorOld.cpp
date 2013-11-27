#include <cstdio>
using namespace std;

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define CAM_STREAM "Camera Stream"
#define THRESH_STREAM "Threshold Stream"
#define ESCAPE_KEY 27
 
IplImage* thresholdImage(IplImage* imgHSV)
{        
    IplImage* imgThresh = cvCreateImage(cvGetSize(imgHSV), IPL_DEPTH_8U, 1);
    cvInRangeS(imgHSV, cvScalar(170,160,60), cvScalar(180,256,256), imgThresh); 
    return imgThresh;
} 

int main()
{
	CvCapture* capture = NULL;
	cvNamedWindow(CAM_STREAM, CV_WINDOW_AUTOSIZE);
	cvNamedWindow(THRESH_STREAM, CV_WINDOW_AUTOSIZE);
 
	while (true)
	{
		capture = cvCaptureFromCAM(1);
		if (!capture)
		{
            printf("Device inaccessible\n");
            return -1;
		}

		IplImage* frame = cvCloneImage(cvQueryFrame(capture));
        cvSmooth(frame, frame, CV_GAUSSIAN, 3, 3);
        IplImage* imgHSV = cvCreateImage(cvGetSize(frame), IPL_DEPTH_8U, 3); 
        cvCvtColor(frame, imgHSV, CV_BGR2HSV);
        IplImage* imgThresh = thresholdImage(imgHSV);
        cvSmooth(imgThresh, imgThresh, CV_GAUSSIAN, 3, 3);

        cvShowImage(THRESH_STREAM, imgThresh);            
        cvShowImage(CAM_STREAM, frame);
            
        cvReleaseImage(&imgThresh);          
		cvReleaseImage(&imgHSV);
        cvReleaseImage(&frame);
		 
		if (cvWaitKey(10) == ESCAPE_KEY)
		{
			break;
		}

        cvReleaseCapture(&capture);
	}
  
	cvDestroyAllWindows();
	
	return 0; 
}


// Hue values of basic colors
// Orange  0-22
// Yellow 22- 38
// Green 38-75
// Blue 75-130
// Violet 130-160
// Red 160-179
