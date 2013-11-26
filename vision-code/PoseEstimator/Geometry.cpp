#include "Geometry.h"

#include <cstdio>
using namespace std; // DEBUG


Mat_<double> estimateRotTransl(
    Mat_<double> const worldPtsHom,
    Mat_<double> const imagePtsHom)
{
    assert(imagePtsHom.cols == 3);
    assert(worldPtsHom.cols == 4);
    assert(imagePtsHom.rows == worldPtsHom.rows);
    // TODO verify all worldPtsHom have z=0

    // See "pose estimation" section in the paper.

    // Set up linear system of equations.
    int const n = imagePtsHom.rows;
    Mat_<double> F(2 * n, 9);
    for(int i = 0; i < n; i++)
    {
        F(2 * i, 0) = worldPtsHom(i, 0);
        F(2 * i, 1) = 0;
        F(2 * i, 2) = -worldPtsHom(i, 0) * imagePtsHom(i, 0);
        F(2 * i, 3) = worldPtsHom(i, 1);
        F(2 * i, 4) = 0;
        F(2 * i, 5) = -worldPtsHom(i, 1) * imagePtsHom(i, 0);
        F(2 * i, 6) = 1;
        F(2 * i, 7) = 0;
        F(2 * i, 8) = -imagePtsHom(i, 0);

        F(2 * i + 1, 0) = 0;
        F(2 * i + 1, 1) = worldPtsHom(i, 0);
        F(2 * i + 1, 2) = -worldPtsHom(i, 0) * imagePtsHom(i, 1);
        F(2 * i + 1, 3) = 0;
        F(2 * i + 1, 4) = worldPtsHom(i, 1);
        F(2 * i + 1, 5) = -worldPtsHom(i, 1) * imagePtsHom(i, 1);
        F(2 * i + 1, 6) = 0;
        F(2 * i + 1, 7) = 1;
        F(2 * i + 1, 8) = -imagePtsHom(i, 1);
    }

    // Find least-squares estimate of rotation + translation.
    SVD svd(F);
    cout << svd.u << endl << svd.w << endl << svd.vt << endl;
}

Mat_<double> estimatePose(Mat_<double> const imagePts)
{
    Mat_<double> imagePtsHom = cartToHom(imagePts);
    Mat_<double> const worldPtsHom = getWorldPtsHom();
    Mat_<double> const pose = estimateRotTransl(worldPtsHom, imagePtsHom);

    Mat_<double> simplePose;
    simplePose(0) = pose(0, 3);  // x
    simplePose(1) = pose(1, 3);  // y
    simplePose(2) = pose(2, 3);  // z
    // Yaw (rotation around z axis):
    // See http://planning.cs.uiuc.edu/node103.html
    // TODO verify that this is correct
    simplePose(3) = atan2(pose(1, 0), pose(0, 0));

    return simplePose;
}

Mat_<double> getWorldPtsHom()
{
    Mat_<double> worldPtsHom(24, 4, CV_64FC1);
    for(int i = 0; i < 24; i++)
    {
        worldPtsHom[i][2] = 0;  // z = 0
        worldPtsHom[i][3] = 1;  // homogeneous coords
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
    worldPtsHom(19, 1) = 200;

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

    // Assuming camera calibration matrix is identity.
    // Note that OpenCV treats size as "[cols rows]", so matrix multiplication
    // has to be done backwards (transpose everything).
    Mat_<double> projection = Mat_<double>::eye(3, 4);
    return projection * rigidMotion * worldPtsHom.t();
}
