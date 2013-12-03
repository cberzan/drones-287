#include <opencv2/opencv.hpp>
using namespace cv;

/**
 * Detect corners of landing pad in the provided image.
 *
 * If succeeds, returns 24 points in the order described in the paper.
 * If fails, returns an empty matrix.
 *
 * Inputs:
 * frame = input image
 *
 * Returns:
 * imagePts (24x2) = image points
 */
Mat_<double> detectCorners(
    Mat const frame,
    char const* inputWindowHandle = NULL,
    char const* cannyWindowHandle = NULL,
    char const* contourWindowHandle = NULL);
