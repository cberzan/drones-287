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
 * worldPts (Nx3) = world points
 * imagePts (Nx2) = image points
 *
 * Returns:
 * pose (3x4) = [3x3] rotation and [3x1] translation of camera w.r.t. world
 */
Mat_<double> estimateRotTransl(
    Mat_<double> const worldPts,
    Mat_<double> const imagePts);

/**
 * Estimate camera pose w.r.t. hardcoded landing pad.
 *
 * Inputs:
 * imagePts (Nx2) = image points in the order described in the paper
 *
 * Returns:
 * pose (4x1) = x, y, z, yaw coords of camera
 */
Mat_<double> estimatePose(Mat_<double> const imagePts);

/**
 * Return landing pad points in the order described in the paper.
 *
 * The coordinates are in millimeters.
 * The origin of the coordinate system is point #3, the pad center.
 *
 * Returns:
 * worldPts (24x3) = world points
 */
Mat_<double> getWorldPts();

/**
 * Convert from Cartesian to homogeneous coordinates.
 *
 * Inputs:
 * cart (NxD) = N points in D dimensions
 *
 * Returns:
 * hom (NxD+1) = N points in homogeneous coordinates.
 */
Mat_<double> cartToHom(Mat_<double> const cart);

/**
 * Convert from homogeneous to Cartesian coordinates.
 *
 * Inputs:
 * hom (NxD+1) = N points in homogeneous coordinates.
 *
 * Returns:
 * cart (NxD) = N points in D dimensions
 */
Mat_<double> homToCart(Mat_<double> const hom);

/**
 * Compute 3D rotation matrix given rotation vector and angle.
 *
 * Inputs:
 * rotAxis (3x1) = unit vector indicating rotation axis
 * rotAngle = amount of rotation in radians
 *
 * Returns:
 * rotation (3x3) = rotation matrix
 */
Mat_<double> rotAxisAngleToRotMatrix(
    Mat_<double> const rotAxis,
    double rotAngle);

/**
 * Project points in the world to points in the image.
 *
 * Everything is in homogeneous coordinates.
 * We assume the camera calibration matrix is identity.
 *
 * Inputs:
 * worldPtsHom (Nx4) = world points in homogeneous coordinates
 * rotMatrix (3x3) = rotation of camera w.r.t. world
 * translation (3x1) = translation of camera w.r.t. world
 *
 * Returns:
 * imagePtsHom (Nx3) = image points in homogeneous coordinates
 */
Mat_<double> worldHomToCameraHom(
    Mat_<double> const worldPtsHom,
    Mat_<double> const rotMatrix,
    Mat_<double> const translation);

/**
 * Draw the landing pad given by imagePts onto the given image.
 *
 * Inputs:
 * imagePts (Nx2) = image points in the order described in the paper
 *
 * Outputs:
 * (modifies image)
 */
void drawImagePts(Mat image, Mat_<double> const imagePts);
