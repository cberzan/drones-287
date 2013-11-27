#include "opencv2/highgui/highgui.hpp"
#include <cv.h>
#include <highgui.h>
#include <iostream>

#define INPUT_FRAMES "Input"
#define CONTOURS "Contours"
#define CANNY "Canny"

#define ESCAPE_KEY 27
 
using namespace cv;
using namespace std;

int main()
{
	namedWindow(INPUT_FRAMES, CV_WINDOW_AUTOSIZE);
	namedWindow(CONTOURS, CV_WINDOW_AUTOSIZE);
	// namedWindow(CANNY, CV_WINDOW_AUTOSIZE);

	Mat frame, cannyImage;

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;

	char* imageNames[] = {"platform1.jpg", "platform2.jpg", "platform3.jpg", NULL};
	int imageIndex = 0;
	while (true)
	{
		frame = imread(imageNames[imageIndex]);
		if (frame.empty())
		{
			cout << "Couldn't load input image" << endl;
			continue;
		}
		imageIndex++;
		if (!imageNames[imageIndex])
			imageIndex = 0;

		GaussianBlur(frame, frame, Size(3,3), 0, 0);		

		Canny(frame, cannyImage, 50, 200, 3);
		// imshow(CANNY, cannyImage);
			
		findContours(cannyImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat contourImg = Mat::zeros(cannyImage.size(), CV_8UC3);
		vector<Point> approx;
		for (int i = 0; i< contours.size(); i++)
		{				
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.02, true);
			if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 100 && isContourConvex(Mat(approx)))
				drawContours(contourImg, contours, i, Scalar(0,0,255), 2, 8, hierarchy, 0, Point());
		}

		imshow(CONTOURS, contourImg);		
		imshow(INPUT_FRAMES, frame);
			
		if ((char)waitKey() == ESCAPE_KEY)
			break;		
	}
	destroyAllWindows();
	return 0; 
}
