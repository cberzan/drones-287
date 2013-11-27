#include "Geometry.h"

#include <cstdio>
using namespace std;

void testHomCoords()
{
    Mat_<double> cart(5, 2);
    cart << 1, 2, 3.3, 4, 5, 6, 7, 8, 9, 10;
    Mat_<double> homCorrect(5, 3);
    homCorrect << 1, 2, 1, 3.3, 4, 1, 5, 6, 1, 7, 8, 1, 9, 10, 1;
    Mat_<double> hom = cartToHom(cart);
    double error = sum(abs(hom - homCorrect))[0];
    assert(error < 1e-14);

    Mat_<double> scale(5, 1);
    scale << 4, 9, 12.3, -3, -5.3;
    Mat_<double> homNew = Mat::diag(scale) * hom;
    Mat_<double> cartNew = homToCart(homNew);
    error = sum(abs(cartNew - cart))[0];
    assert(error < 1e-14);
}

void testWorldPts()
{
    Mat_<double> worldPts = getWorldPts();
    Mat_<double> imagePts = worldPts(Range::all(), Range(0, 2));
    /*
    Mat image = Mat::zeros(600, 600, CV_8UC3);
    drawImagePts(image, imagePts);
    imshow("getWorldPts", image);
    waitKey(0);
    */
}

void testRotMatrix()
{
    // Test 1: rotating along z axis:
    Mat_<double> s(3, 1);
    s << 0, 0, 1;
    double phi = 0.4;
    Mat_<double> expected_R(3, 3);
    expected_R << cos(phi), -sin(phi), 0, \
                    sin(phi), cos(phi), 0, \
                    0, 0, 1;
    Mat_<double> R = rotAxisAngleToRotMatrix(s, phi);
    double error = sum(abs(R - expected_R))[0];
    assert(error < 1e-10);

    // Test 2: rotating around an arbitrary axis by 360 degrees:
    s(0) = 12;
    s(1) = 34;
    s(2) = 56;
    s = s / norm(s);
    phi = 2 * M_PI;
    expected_R = Mat_<double>::eye(3, 3);
    R = rotAxisAngleToRotMatrix(s, phi);
    error = sum(abs(R - expected_R))[0];
    assert(error < 1e-10);
}

void testSyntheticRotation()
{
    // Synthetic rotation + translation of the camera:
    Mat_<double> translation(3, 1);
    translation << 0, -50, 100;
    Mat_<double> rotAxis(3, 1);
    rotAxis << 0, 0, 1;
    rotAxis /= norm(rotAxis);
    double rotAngle = 30 * M_PI / 180;
    Mat_<double> rotMatrix = rotAxisAngleToRotMatrix(rotAxis, rotAngle);
    Mat_<double> worldPts = getWorldPts();
    Mat_<double> worldPtsHom = cartToHom(worldPts);
    Mat_<double> imagePtsHom = worldHomToCameraHom(
        worldPtsHom, rotMatrix, translation);
    Mat_<double> imagePts = homToCart(imagePtsHom);
    cout << "true translation: " << translation << endl;
    cout << "true yaw angle: " << rotAngle << endl;
    cout << "true rotation matrix: " << rotMatrix << endl;
    // cout << imagePtsHom << endl << imagePts << endl;

    /*
    Mat image = Mat::zeros(600, 600, CV_8UC3);
    drawImagePts(image, imagePts);
    imshow("Test", image);
    waitKey(0);
    */

    Mat_<double> rotTransl = estimateRotTransl(worldPts, imagePts);
    Mat_<double> gotRot = rotTransl(Range(0, 3), Range(0, 3));
    Mat_<double> gotTransl = rotTransl.col(3);
    double translErr = sum(abs(translation - gotTransl))[0];
    double rotErr = sum(abs(rotMatrix - gotRot))[0];
    cout << "translErr=" << translErr << ", rotErr=" << rotErr << endl;
    assert(translErr < 1e-10);
    assert(rotErr < 1e-10);
}

int main()
{
    testHomCoords();
    testWorldPts();
    testRotMatrix();
    testSyntheticRotation();
}
