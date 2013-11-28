#include <opencv2/opencv.hpp>
using namespace cv;

/**
 * Capture a frame from the camera and detect landing pad corners.
 *
 * If succeeds, returns true and fills imagePts.
 * If fails, returns false and leaves imagePts untouched.
 *
 * Outputs:
 * imagePts (24x2) = image points
 *
 * Returns:
 * ok = true if all corners detected, false otherwise
 */
bool captureAndDetectCorners(Mat_<double> &imagePts);
