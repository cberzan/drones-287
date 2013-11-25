#include <opencv2/opencv.hpp>
using namespace cv;

/**
 * Estimate rotation and translation of camera w.r.t. world.
 *
 * Takes a point correspondence between worldPts and imagePts. Computes the
 * rotation and translation of the camera coordinate frame w.r.t. the world
 * coordinate frame. The worldPts are expected to have z=0.
 *
 * Inputs:
 * worldPts (Nx4) = world points in homogeneous coordinates
 * imagePts (Nx3) = image points in homogeneous coordinates
 *
 * Outputs:
 * rotation (3x3) = rotation of camera w.r.t. world
 * translation (3x1) = translation of camera w.r.t. world
 */
void estimateRotationTranslation(
    Mat const worldPts,
    Mat const imagePts,
    Mat rotation,
    Mat translation);

/**
 * Estimate camera pose w.r.t. hardcoded landing pad.
 *
 * Inputs:
 * imagePts (Nx2) = image points in the order described in the paper
 *
 * Outputs:
 * pose (3x1) = x, y, z coords of camera
 */
void estimatePose(Mat const imagePts, Mat pose);
