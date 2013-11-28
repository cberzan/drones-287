#include "Corners.h"

bool captureAndDetectCorners(Mat_<double> &imagePts)
{
    Mat_<double> tmp(24, 2);

    // XXX dummy
    for(int i = 0; i < 24; i++) {
        tmp(i, 0) = i;
        tmp(i, 1) = -i;
    }

    // This always does a copy, which is inefficient.
    // I can't figure out how to resize imagePts to the size I need,
    // so I can write to it directly.
    tmp.copyTo(imagePts);
    return true;
}
