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

int getIndexOfMax(int (&Arr)[50], size_t size)
{
	int max = 0, indexOfMax = 0;
	for (int i=0; i<size; i++)
	{
		if (Arr[i] > max)
		{
			max = Arr[i];
			indexOfMax = i;
		}
	}
	return indexOfMax;
}

int main()
{
	namedWindow(INPUT_FRAMES, CV_WINDOW_AUTOSIZE);
	namedWindow(CONTOURS, CV_WINDOW_AUTOSIZE);
	// namedWindow(CANNY, CV_WINDOW_AUTOSIZE);

	Mat frame, cannyImage, filteredImage;

	vector<Vec4i> hierarchy;
	vector<vector<Point> > contours;

	char* imageNames[] = {"platform1.jpg", "platform2.jpg", "platform3.jpg", NULL};
	int imageIndex = 0;
	while (true)
	{
		frame = imread("../" + String(imageNames[imageIndex]));		
		if (frame.empty())
		{
			cout << "Couldn't load input image" << endl;
			continue;
		}
		imageIndex++;
		if (imageNames[imageIndex] == NULL)
			imageIndex = 0;

		// GaussianBlur(frame, frame, Size(3,3), 0, 0);		

		medianBlur(frame, filteredImage, 3);
		Canny(filteredImage, cannyImage, 50, 200, 3);
		// imshow(CANNY, cannyImage);
			
		findContours(cannyImage, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, Point(0, 0));

		Mat contourImg = Mat::zeros(cannyImage.size(), CV_8UC3);
		vector<Point> approx;

		int parentContours[50] = {0}, selectedContours[50] = {0}, vContourCount = 0;
		for (int i=0; i<contours.size(); i++)
		{	
			if (hierarchy[i].val[3] == -1)
				continue;
			approxPolyDP(Mat(contours[i]), approx, arcLength(Mat(contours[i]), true)*0.03, true);
			if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 100 && isContourConvex(Mat(approx)))
			{
				parentContours[hierarchy[i].val[3]]++;
				selectedContours[vContourCount] = i;
				vContourCount++;
			}
		}

		int indexOfOuterSquare = getIndexOfMax(parentContours, contours.size());
		// If all squares are not detected, move on to the next frame
		if (parentContours[indexOfOuterSquare] != 6)
			continue;

		int cIndex;
		for (int i=0; i<vContourCount; i++)
		{	
			cIndex = selectedContours[i];
			if (hierarchy[cIndex].val[3] != indexOfOuterSquare)
				continue;
			approxPolyDP(Mat(contours[cIndex]), approx, arcLength(Mat(contours[cIndex]), true)*0.03, true);
			if (approx.size() == 4 && fabs(contourArea(Mat(approx))) > 100 && isContourConvex(Mat(approx)))
			{
				// To-do: Fix Labeling
				putText(contourImg, "0", approx[0], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
				putText(contourImg, "1", approx[1], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
				putText(contourImg, "2", approx[2], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
				putText(contourImg, "3", approx[3], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
				drawContours(contourImg, contours, cIndex, Scalar(255,0,0), 1, 8, hierarchy, 0, Point());
			}
		}

		imshow(CONTOURS, contourImg);		
		imshow(INPUT_FRAMES, frame);
			
		if ((char)waitKey(0) == ESCAPE_KEY)
			break;		
	}
	destroyAllWindows();
	return 0; 
}
