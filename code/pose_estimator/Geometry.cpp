#include "Geometry.h"

#include <cstdio>
using namespace std; // DEBUG


Mat_<double> estimateRotTransl(
    Mat_<double> const worldPts,
    Mat_<double> const imagePts)
{
    assert(imagePts.cols == 2);
    assert(worldPts.cols == 3);
    assert(imagePts.rows == worldPts.rows);
    // TODO verify all worldPts have z=0

    // See "pose estimation" section in the paper.

    // Set up linear system of equations.
    int const n = imagePts.rows;
    Mat_<double> F(2 * n, 9);
    for(int i = 0; i < n; i++)
    {
        F(2 * i, 0) = worldPts(i, 0);
        F(2 * i, 1) = 0;
        F(2 * i, 2) = -worldPts(i, 0) * imagePts(i, 0);
        F(2 * i, 3) = worldPts(i, 1);
        F(2 * i, 4) = 0;
        F(2 * i, 5) = -worldPts(i, 1) * imagePts(i, 0);
        F(2 * i, 6) = 1;
        F(2 * i, 7) = 0;
        F(2 * i, 8) = -imagePts(i, 0);

        F(2 * i + 1, 0) = 0;
        F(2 * i + 1, 1) = worldPts(i, 0);
        F(2 * i + 1, 2) = -worldPts(i, 0) * imagePts(i, 1);
        F(2 * i + 1, 3) = 0;
        F(2 * i + 1, 4) = worldPts(i, 1);
        F(2 * i + 1, 5) = -worldPts(i, 1) * imagePts(i, 1);
        F(2 * i + 1, 6) = 0;
        F(2 * i + 1, 7) = 1;
        F(2 * i + 1, 8) = -imagePts(i, 1);
    }

    // Find least-squares estimate of rotation + translation.
    SVD svd(F);

    Mat_<double> rrp = svd.vt.row(8);
    rrp = rrp.clone().reshape(0, 3).t();
    if(rrp(2, 2) < 0) {
        rrp *= -1;  // make sure depth is positive
    }
    // cout << "rrp: " << rrp << endl;

    Mat_<double> transl = \
        2 * rrp.col(2) / (norm(rrp.col(0)) + norm(rrp.col(1)));
    // cout << "transl: " << transl << endl;

    Mat_<double> rot = Mat_<double>::zeros(3, 3);
    rrp.col(0).copyTo(rot.col(0));
    rrp.col(1).copyTo(rot.col(1));
    SVD svd2(rot);
    rot = svd2.u * svd2.vt;
    if(determinant(rot) < 0) {
        rot.col(2) *= -1; // make sure it's a valid rotation matrix
    }
    if(abs(determinant(rot) - 1) > 1e-10) {
        cerr << "Warning: rotation matrix has determinant " \
             << determinant(rot) << " where expected 1." << endl;
    }
    // cout << "rot: " << rot << endl;

    Mat_<double> rotTransl(3, 4);
    rot.col(0).copyTo(rotTransl.col(0));
    rot.col(1).copyTo(rotTransl.col(1));
    rot.col(2).copyTo(rotTransl.col(2));
    transl.copyTo(rotTransl.col(3));
    return rotTransl;
}

Mat_<double> estimatePose(Mat_<double> const imagePts)
{
    // Note that given R, t, the camera location in world
    // coordinates is not just t, but instead -inv(R)*t.

    Mat_<double> const worldPts = getWorldPts();
    Mat_<double> const rotTransl = estimateRotTransl(worldPts, imagePts);
    Mat_<double> rotMatrix(3, 3);
    Mat_<double> translation(3, 1);
    rotTransl.col(0).copyTo(rotMatrix.col(0));
    rotTransl.col(1).copyTo(rotMatrix.col(1));
    rotTransl.col(2).copyTo(rotMatrix.col(2));
    rotTransl.col(3).copyTo(translation);
    Mat_<double> cameraLoc = -rotMatrix.t() * translation;
    Mat_<double> simplePose(4, 1);
    simplePose(0) = cameraLoc(0);  // x
    simplePose(1) = cameraLoc(1);  // y
    simplePose(2) = cameraLoc(2);  // z
    // Yaw (rotation around z axis):
    // See http://planning.cs.uiuc.edu/node103.html
    simplePose(3) = atan2(rotMatrix(1, 0), rotMatrix(0, 0));
    // cout << "simplePose: " << simplePose << endl;
    return simplePose;
}

Mat_<double> getWorldPts()
{
    Mat_<double> worldPtsHom(24, 3, CV_64FC1);
    for(int i = 0; i < 24; i++) {
        worldPtsHom[i][2] = 0;  // z = 0
    }

    // Square A:
    worldPtsHom(0, 0) = 200;
    worldPtsHom(0, 1) = 200;
    worldPtsHom(1, 0) = 0;
    worldPtsHom(1, 1) = 200;
    worldPtsHom(2, 0) = 0;
    worldPtsHom(2, 1) = 0;
    worldPtsHom(3, 0) = 200;
    worldPtsHom(3, 1) = 0;

    // Square B:
    worldPtsHom(4, 0) = -120;
    worldPtsHom(4, 1) = 200;
    worldPtsHom(5, 0) = -200;
    worldPtsHom(5, 1) = 200;
    worldPtsHom(6, 0) = -200;
    worldPtsHom(6, 1) = 120;
    worldPtsHom(7, 0) = -120;
    worldPtsHom(7, 1) = 120;

    // Squre C:
    worldPtsHom(8, 0) = -120;
    worldPtsHom(8, 1) = 40;
    worldPtsHom(9, 0) = -200;
    worldPtsHom(9, 1) = 40;
    worldPtsHom(10, 0) = -200;
    worldPtsHom(10, 1) = -40;
    worldPtsHom(11, 0) = -120;
    worldPtsHom(11, 1) = -40;

    // Squre D:
    worldPtsHom(12, 0) = -120;
    worldPtsHom(12, 1) = -120;
    worldPtsHom(13, 0) = -200;
    worldPtsHom(13, 1) = -120;
    worldPtsHom(14, 0) = -200;
    worldPtsHom(14, 1) = -200;
    worldPtsHom(15, 0) = -120;
    worldPtsHom(15, 1) = -200;

    // Squre E:
    worldPtsHom(16, 0) = 40;
    worldPtsHom(16, 1) = -120;
    worldPtsHom(17, 0) = -40;
    worldPtsHom(17, 1) = -120;
    worldPtsHom(18, 0) = -40;
    worldPtsHom(18, 1) = -200;
    worldPtsHom(19, 0) = 40;
    worldPtsHom(19, 1) = -200;

    // Squre F:
    worldPtsHom(20, 0) = 200;
    worldPtsHom(20, 1) = -120;
    worldPtsHom(21, 0) = 120;
    worldPtsHom(21, 1) = -120;
    worldPtsHom(22, 0) = 120;
    worldPtsHom(22, 1) = -200;
    worldPtsHom(23, 0) = 200;
    worldPtsHom(23, 1) = -200;

    return worldPtsHom;
}

Mat_<double> cartToHom(Mat_<double> const cart)
{
    Mat_<double> hom(cart.rows, cart.cols + 1, 1.0);
    Mat_<double> tmp = hom(Range::all(), Range(0, cart.cols));
    cart.copyTo(tmp);
    return hom;
}

Mat_<double> homToCart(Mat_<double> const hom)
{
    // Don't know how to do this efficiently in OpenCV.
    Mat_<double> cart(hom.rows, hom.cols - 1);
    for(int r = 0; r < hom.rows; r++) {
        for(int c = 0; c < hom.cols - 1; c++) {
            cart(r, c) = hom(r, c) / hom(r, hom.cols - 1);
        }
    }
    return cart;
}

Mat_<double> rotAxisAngleToRotMatrix(
    Mat_<double> const rotAxis,
    double rotAngle)
{
    assert(rotAxis.rows == 3);
    assert(rotAxis.cols == 1);
    assert(abs(norm(rotAxis) - 1) < 1e-10);

    // Skew-symmetric matrix representing cross product by s:
    Mat_<double> sHat(3, 3);
    sHat << 0, -rotAxis(2), rotAxis(1), \
            rotAxis(2), 0, -rotAxis(0), \
            -rotAxis(1), rotAxis(0), 0;

    // Roderiques formula:
    Mat_<double> rotation = Mat_<double>::eye(3,3) \
        + sin(rotAngle) * sHat \
        + (1 - cos(rotAngle)) * sHat * sHat;
    return rotation;
}

Mat_<double> worldHomToCameraHom(
    Mat_<double> const worldPtsHom,
    Mat_<double> const rotMatrix,
    Mat_<double> const translation)
{
    assert(worldPtsHom.cols == 4);
    assert(rotMatrix.rows == 3);
    assert(rotMatrix.cols == 3);
    assert(translation.rows == 3);
    assert(translation.cols == 1);

    // Convert rotMatrix + translation into a linear transformation in
    // homogeneous coordinates.
    Mat_<double> rigidMotion = Mat_<double>::zeros(4, 4);
    rotMatrix.copyTo(rigidMotion(Range(0, 3), Range(0, 3)));
    translation.copyTo(rigidMotion(Range(0, 3), Range(3, 4)));
    rigidMotion(3, 3) = 1;
    // cout << "rigidMotion: " << rigidMotion << endl;

    // Assuming camera calibration matrix is identity.
    // Note that OpenCV treats size as "[cols rows]", so matrix multiplication
    // has to be done backwards (transpose everything).
    Mat_<double> projection = Mat_<double>::eye(3, 4);
    Mat result = projection * rigidMotion * worldPtsHom.t();
    return result.t();
}

void drawImagePts(Mat image, Mat_<double> const imagePts)
{
    assert(imagePts.rows == 24);
    assert(imagePts.cols == 2);
    double minVal, maxVal;
    minMaxIdx(imagePts, &minVal, &maxVal);
    double scale = 1.0 * image.rows / (maxVal - minVal);
    Point points[6][4];
    for(int i = 0; i < 24; i++) {
        points[i / 4][i % 4] = Point(
            scale * (imagePts(i, 0) - minVal),
            scale * (imagePts(i, 1) - minVal));
    }
    Point const* points2[] = {
        points[0], points[1], points[2],
        points[3], points[4], points[5]
    };  // stupid fillPoly won't acccept points directly
    int numPoints[] = { 4, 4, 4, 4, 4, 4 };
    fillPoly(image, points2, numPoints, 6, Scalar(255, 255, 255));

    // LEFT TODO: draw a rectangle around everything, otherwise it's hard to
    // tell what's going on...
}

Mat_<double> getCameraMatrix()
{
    Mat_<double> cameraMatrix(3, 3);
    cameraMatrix << 620,   0, 320,
                      0, 620, 240,
                      0,   0,   1;
    return cameraMatrix;
}

Mat_<double> calibrateImagePoints(Mat_<double> const imagePts)
{
    Mat_<double> const cameraMatrix = getCameraMatrix();
    assert(imagePts.cols == 2);
    Mat_<double> calibratedImagePts(imagePts.rows, 2);
    for(int i = 0; i < imagePts.rows; i++) {
        calibratedImagePts(i, 0) = \
            (imagePts(i, 0) - cameraMatrix(0, 2)) / cameraMatrix(0, 0);
        calibratedImagePts(i, 1) = \
            (imagePts(i, 1) - cameraMatrix(1, 2)) / cameraMatrix(1, 1);
    }
    return calibratedImagePts;
}

Mat_<double> unCalibrateImagePoints(Mat_<double> const calibratedImagePts)
{
    Mat_<double> const cameraMatrix = getCameraMatrix();
    assert(calibratedImagePts.cols == 2);
    Mat_<double> imagePts(calibratedImagePts.rows, 2);
    for(int i = 0; i < imagePts.rows; i++) {
        imagePts(i, 0) = \
            calibratedImagePts(i, 0) * cameraMatrix(0, 0) + cameraMatrix(0, 2);
        imagePts(i, 1) = \
            calibratedImagePts(i, 1) * cameraMatrix(1, 1) + cameraMatrix(1, 2);
    }
    return imagePts;
}
