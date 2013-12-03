#include "Geometry.h"

#include <cmath>
#include <cstdio>
#include <iostream>
using namespace std;

// Sample from Normal distribution.
// See https://en.wikipedia.org/wiki/Normal_distribution#Generating_values_from_normal_distribution
// Not a very efficient implementation.
double randomNormal(double mean, double variance)
{
    double u = rand() * 1.0 / RAND_MAX;
    double v = rand() * 1.0 / RAND_MAX;
    double z = sqrt(-2 * log(u)) * cos(2 * M_PI * v);
    return mean + sqrt(variance) * z;
}

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

void testPoseEstimationExact()
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
    // cout << "true translation: " << translation << endl;
    // cout << "true yaw angle: " << rotAngle << endl;
    // cout << "true rotation matrix: " << rotMatrix << endl;
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
    // cout << "translErr=" << translErr << ", rotErr=" << rotErr << endl;
    assert(translErr < 1e-10);
    assert(rotErr < 1e-10);

    Mat_<double> pose = estimatePose(imagePts);
    gotTransl = pose(Range(0, 3), Range::all());
    double gotYaw = pose(3);
    translErr = sum(abs(translation - gotTransl))[0];
    double yawErr = abs(gotYaw - rotAngle);
    // cout << "translErr=" << translErr << ", yawErr=" << yawErr << endl;
    assert(yawErr < 1e-10);  // assumes rotating around z axis
}

void testNormal()
{
    int n = 1000000;
    double mean = 100.0, variance = 20;
    Mat_<double> samples(n, 1);
    srand(666);
    for(int i = 0; i < n; i++) {
        samples(i) = randomNormal(mean, variance);
    }
    Scalar gotMean, gotStdDev;
    meanStdDev(samples, gotMean, gotStdDev);
    double meanErr = abs(mean - gotMean[0]);
    double varianceErr = abs(variance - gotStdDev[0] * gotStdDev[0]);
    // cout << "meanErr=" << meanErr << "; varianceErr=" << varianceErr << endl;
    assert(meanErr < 0.01);
    assert(varianceErr < 0.05);
}

void testPoseEstimationNoisy()
{
    // The idea here is to test that our pose estimation is immune to noise.
    // We add Gaussian noise to the image points and compare the estimate of
    // the translation + rotation to the ground truth.
    //
    // The noise parameters and error tolerances I pulled out out of thin air.

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
    // cout << "true translation: " << translation << endl;
    // cout << "true yaw angle: " << rotAngle << endl;
    // cout << "true rotation matrix: " << rotMatrix << endl;
    // cout << "imagePts: " << imagePts << endl;

    Scalar imagePtsMean, imagePtsStdDev;
    meanStdDev(imagePts.reshape(0, 1), imagePtsMean, imagePtsStdDev);
    // cout << "mean: " << imagePtsMean[0] << "; stdev: " << imagePtsStdDev[0] << endl;
    double const mean = 0.0;
    double const variance = 0.01;
    int trials = 100;
    int trialsGood = 0;
    for(int i = 0; i < trials; i++) {
        Mat_<double> noisyImagePts = imagePts.clone();
        srand(666 + i);
        for(int j = 0; j < noisyImagePts.rows; j++) {
            noisyImagePts(j, 0) += randomNormal(mean, variance);
            noisyImagePts(j, 1) += randomNormal(mean, variance);
        }

        /*
        Mat image = Mat::zeros(600, 600, CV_8UC3);
        drawImagePts(image, noisyImagePts);
        imshow("Test", image);
        waitKey(0);
        */

        Mat_<double> rotTransl = estimateRotTransl(worldPts, noisyImagePts);
        Mat_<double> gotRot = rotTransl(Range(0, 3), Range(0, 3));
        Mat_<double> gotTransl = rotTransl.col(3);
        // cout << "gotRot: " << gotRot << endl;
        // cout << "gotTransl: " << gotTransl << endl;
        double translErr = sum(abs(translation - gotTransl))[0];
        double rotErr = sum(abs(rotMatrix - gotRot))[0];
        // cout << "translErr=" << translErr << ", rotErr=" << rotErr << endl;
        bool good = true;
        if(translErr >= 12) {
            // cout << "bad translErr" << endl;
            good = false;
        }
        if(rotErr > 0.2) {
            // cout << "bad rotErr" << endl;
            good = false;
        }

        Mat_<double> pose = estimatePose(noisyImagePts);
        gotTransl = pose(Range(0, 3), Range::all());
        double gotYaw = pose(3);
        // cout << "gotYaw=" << gotYaw << endl;
        translErr = sum(abs(translation - gotTransl))[0];
        double yawErr = abs(gotYaw - rotAngle);
        // cout << "translErr=" << translErr << ", yawErr=" << yawErr << endl;
        if(yawErr >= 0.05) {
            // cout << "bad yawErr" << endl;
            good = false;
        }
        if(good) {
            trialsGood++;
        }
    }
    cout << "Within tolerance " << trialsGood
         << " of " << trials << " times." << endl;
    assert(1.0 * trialsGood / trials >= 0.9);
}

int main()
{
    testHomCoords();
    testWorldPts();
    testRotMatrix();
    testPoseEstimationExact();
    testNormal();
    testPoseEstimationNoisy();
    cout << "Tests passed." << endl;
}
