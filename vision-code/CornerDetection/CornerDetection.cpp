#include <cstdio>

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define INPUT_FRAMES "Input"
#define CONTOURS "Contours"
#define CANNY "Canny"
#define ESCAPE_KEY 27

using namespace cv;
using namespace std;

class _Polygon
{
public:
    char Label[2];
    vector<Point> Corners;
    Point2f Center;

    _Polygon()
    {
        Label[0] = '-';
        Label[1] = '\0';
    }
};

struct _Diagonal
{
    Vec2f Diag;
    int indexOfCorner1;
    int indexOfCorner2;
};

inline float sqr(float x)
{
    return x*x;
}

inline float calcDistanceBet2Points(Point2f P1, Point2f P2)
{
    return sqrt(sqr(P1.x - P2.x) + sqr(P1.y - P2.y));
}

void labelPolygons(_Polygon (&Polygons)[6], int (&orderOfPolygons)[6]);
void drawPolygonLabels(Mat & contourImg, _Polygon (&Polygons)[6], int (&orderOfPolygons)[6]);
void labelCorners(_Polygon (&Polygons)[6], int (&orderOfPolygons)[6], Point2f (&Corners)[24]);
void drawCornerLabels(Mat & contourImg, Point2f (&Corners)[24]);
int getIndexOfOuterSquare(int (&Arr)[50], size_t size);

int main()
{
    VideoCapture capture(0);
    if (!capture.isOpened())
    {
        printf("Device inaccessible. Damn!.\n");
        return -1;
    }

    namedWindow(INPUT_FRAMES, CV_WINDOW_AUTOSIZE);
    namedWindow(CONTOURS, CV_WINDOW_AUTOSIZE);
    // namedWindow(CANNY, CV_WINDOW_AUTOSIZE);

    Mat frame, cannyImage, filteredImage;

    vector<Vec4i> hierarchy;
    vector<vector<Point> > contours;

    while (true)
    {
        capture >> frame;
        if (frame.empty())
        {
            cout << "Couldn't load input image" << endl;
            continue;
        }

        // GaussianBlur(frame, filteredImage, Size(3,3), 0, 0);
        // medianBlur(filteredImage, filteredImage, 3);

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

        int indexOfOuterSquare = getIndexOfOuterSquare(parentContours, contours.size());
        // If all squares are not detected, move on to the next frame
        if (parentContours[indexOfOuterSquare] != 6)
            continue;

        int cIndex, polygonIndex = 0;
        double maxPolyArea = 0, PolyArea = 0;
        _Polygon Polygons[6];
        Moments mo;

        int orderOfPolygons[6] = {0};
        for (int i=0; i<vContourCount; i++)
        {
            cIndex = selectedContours[i];
            if (hierarchy[cIndex].val[3] != indexOfOuterSquare)
                continue;
            approxPolyDP(Mat(contours[cIndex]), approx, arcLength(Mat(contours[cIndex]), true)*0.03, true);
            PolyArea = fabs(contourArea(Mat(approx)));
            if (approx.size() == 4 && PolyArea > 100 && isContourConvex(Mat(approx)))
            {
                Polygons[polygonIndex].Corners = approx;
                mo = moments(contours[cIndex], false);
                Polygons[polygonIndex].Center = Point2f(mo.m10/mo.m00 , mo.m01/mo.m00);
                if (PolyArea > maxPolyArea)
                {
                    maxPolyArea = PolyArea;
                    orderOfPolygons[0] = polygonIndex;
                }

                drawContours(contourImg, contours, cIndex, Scalar(255,0,0), 1, 8, hierarchy, 0, Point());
                polygonIndex++;
            }
        }

        labelPolygons(Polygons, orderOfPolygons);
        drawPolygonLabels(contourImg, Polygons, orderOfPolygons);

        Point2f Corners[24];
        labelCorners(Polygons, orderOfPolygons, Corners);
        drawCornerLabels(contourImg, Corners);

        imshow(CONTOURS, contourImg);
        imshow(INPUT_FRAMES, frame);

        if ((char)waitKey(0) == ESCAPE_KEY)
            break;
    }
    destroyAllWindows();
    return 0;
}


void labelPolygons(_Polygon (&Polygons)[6], int (&orderOfPolygons)[6])
{
    float distanceFromAToD = 0, dist;
    for (int k=0; k<6; k++)
    {
        if (k != orderOfPolygons[0])
        {
            dist = calcDistanceBet2Points(Polygons[orderOfPolygons[0]].Center, Polygons[k].Center);
            if (dist > distanceFromAToD)
            {
                distanceFromAToD = dist;
                orderOfPolygons[3] = k;
            }
        }
    }

    Vec3f DA = Vec3f(Polygons[orderOfPolygons[0]].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                    Polygons[orderOfPolygons[0]].Center.y - Polygons[orderOfPolygons[3]].Center.y,
                    0);

    float distFromD, pCount = 0, nCount = 0, pDist = 0, nDist = 0;
    for (int k=0; k<6; k++)
    {
        if (k == orderOfPolygons[3] || k == orderOfPolygons[0])
            continue;
        Vec3f DV = Vec3f(Polygons[k].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                    Polygons[k].Center.y - Polygons[orderOfPolygons[3]].Center.y,
                    0);

        distFromD = calcDistanceBet2Points(Polygons[orderOfPolygons[3]].Center, Polygons[k].Center);

        Vec3f cP = DA.cross(DV);
        if (cP.val[2] > 0)
        {
            if (pCount == 0)
            {
                pDist = distFromD;
                orderOfPolygons[5] = k;
                pCount++;
            }
            else
            {
                if (pDist < distFromD)
                {
                    orderOfPolygons[4] = orderOfPolygons[5];
                    orderOfPolygons[5] = k;
                }
                else
                {
                    orderOfPolygons[4] = k;
                }
            }
        }
        else
        {
            if (nCount == 0)
            {
                nDist = distFromD;
                orderOfPolygons[1] = k;
                nCount++;
            }
            else
            {
                if (nDist < distFromD)
                {
                    orderOfPolygons[2] = orderOfPolygons[1];
                    orderOfPolygons[1] = k;
                }
                else
                {
                    orderOfPolygons[2] = k;
                }
            }
        }
    }
}

void drawPolygonLabels(Mat & contourImg, _Polygon (&Polygons)[6], int (&orderOfPolygons)[6])
{
    Polygons[orderOfPolygons[0]].Label[0] = 'A';
    Polygons[orderOfPolygons[1]].Label[0] = 'B';
    Polygons[orderOfPolygons[2]].Label[0] = 'C';
    Polygons[orderOfPolygons[3]].Label[0] = 'D';
    Polygons[orderOfPolygons[4]].Label[0] = 'E';
    Polygons[orderOfPolygons[5]].Label[0] = 'F';

    putText(contourImg, Polygons[orderOfPolygons[0]].Label, Polygons[orderOfPolygons[0]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    putText(contourImg, Polygons[orderOfPolygons[1]].Label, Polygons[orderOfPolygons[1]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    putText(contourImg, Polygons[orderOfPolygons[2]].Label, Polygons[orderOfPolygons[2]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    putText(contourImg, Polygons[orderOfPolygons[3]].Label, Polygons[orderOfPolygons[3]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    putText(contourImg, Polygons[orderOfPolygons[4]].Label, Polygons[orderOfPolygons[4]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    putText(contourImg, Polygons[orderOfPolygons[5]].Label, Polygons[orderOfPolygons[5]].Center, FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
}

void labelCorners(_Polygon (&Polygons)[6], int (&orderOfPolygons)[6], Point2f (&Corners)[24])
{
    Vec2f DA = Vec2f(Polygons[orderOfPolygons[0]].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                        Polygons[orderOfPolygons[0]].Center.y - Polygons[orderOfPolygons[3]].Center.y);
    DA = normalize(DA);

    Vec3f DA3;
    DA3.val[0] = DA.val[0];
    DA3.val[1] = DA.val[1];
    DA3.val[2] = 0;

    _Diagonal Diagonal;
    float angle, dist, dist2;
    int startingPoint[6], cornerLabel = 1, cornerIndex;
    for (int polygonIndex = 0; polygonIndex < 6; polygonIndex++)
    {
        Diagonal.Diag = Vec2f(Polygons[polygonIndex].Corners[0].x - Polygons[polygonIndex].Corners[2].x,
                            Polygons[polygonIndex].Corners[0].y - Polygons[polygonIndex].Corners[2].y);
        Diagonal.indexOfCorner1 = 0;
        Diagonal.indexOfCorner2 = 2;
        angle = DA.dot(normalize(Diagonal.Diag));
        if (abs(angle) < 0.95)
        {
            Diagonal.Diag = Vec2f(Polygons[polygonIndex].Corners[1].x - Polygons[polygonIndex].Corners[3].x,
                                    Polygons[polygonIndex].Corners[1].y - Polygons[polygonIndex].Corners[3].y);
            Diagonal.indexOfCorner1 = 1;
            Diagonal.indexOfCorner2 = 3;
        }
        startingPoint[polygonIndex] = Diagonal.indexOfCorner1;
        // For square A
        if (polygonIndex == orderOfPolygons[0])
        {
            // Select the corner which is farther from the center of D
            dist = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1], Polygons[orderOfPolygons[3]].Center);
            dist2 = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner2], Polygons[orderOfPolygons[3]].Center);
            if (dist2 > dist)
                startingPoint[polygonIndex] = Diagonal.indexOfCorner2;
        }
        // For square D
        else if (polygonIndex == orderOfPolygons[3])
        {
            // Select the corner which is closer to the center of A
            dist = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1], Polygons[orderOfPolygons[0]].Center);
            dist2 = calcDistanceBet2Points(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner2], Polygons[orderOfPolygons[0]].Center);
            if (dist2 < dist)
                startingPoint[polygonIndex] = Diagonal.indexOfCorner2;
        }
        // For remaining squares
        else
        {
            // Consider vector DV as the vector joining D's center to the current polygon's center.
            // Select the corner which is on the same side of DV as that of A's center.
            Vec3f DV = Vec3f(Polygons[polygonIndex].Center.x - Polygons[orderOfPolygons[3]].Center.x,
                            Polygons[polygonIndex].Center.y - Polygons[orderOfPolygons[3]].Center.y,
                            0);
            Vec3f cP1 = DV.cross(DA3);

            Vec3f DCorner = Vec3f(Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1].x - Polygons[orderOfPolygons[3]].Center.x,
                                    Polygons[polygonIndex].Corners[Diagonal.indexOfCorner1].y - Polygons[orderOfPolygons[3]].Center.y,
                                    0);
            Vec3f cP2 = DV.cross(DCorner);

            if (cP1.val[2]*cP2.val[2] < -1)
                startingPoint[polygonIndex] = Diagonal.indexOfCorner2;
        }
    }

    for (int polygonIndex = 0; polygonIndex < 6; polygonIndex++)
    {
        cornerIndex = startingPoint[orderOfPolygons[polygonIndex]];
        for (int k=0; k<4; k++)
        {
            Corners[cornerLabel-1] = Polygons[orderOfPolygons[polygonIndex]].Corners[cornerIndex];
            cornerLabel++;
            cornerIndex = (cornerIndex + 1) % 4;
        }
    }
}

void drawCornerLabels(Mat & contourImg, Point2f (&Corners)[24])
{
    // Display corner labels
    char cLabel[3];
    for (int i=1; i<=24; i++)
    {
        sprintf(cLabel, "%d", i);
        putText(contourImg, cLabel, Corners[i-1], FONT_HERSHEY_PLAIN, 1, Scalar(255,255,255));
    }
}

int getIndexOfOuterSquare(int (&Arr)[50], size_t size)
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
