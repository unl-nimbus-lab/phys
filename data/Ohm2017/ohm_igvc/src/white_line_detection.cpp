#include "opencv2/imgproc.hpp"
#include "opencv2/highgui.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdlib.h>
#include <ros/ros.h>
#include "geometry_msgs/Point32.h"
#include "ohm_igvc/pixel_locations.h"
#define timeStart //start = clock();
#define timeEnd //cout << "time: " << (clock() - start) / (double)(CLOCKS_PER_SEC / 1000) << "ms" << endl;

using namespace std;
using namespace cv;
void on_low_H_thresh_trackbar(int, void *);
void on_high_H_thresh_trackbar(int, void *);
void on_low_S_thresh_trackbar(int, void *);
void on_high_S_thresh_trackbar(int, void *);
void on_low_V_thresh_trackbar(int, void *);
void on_high_V_thresh_trackbar(int, void *);
double A, B, C, D;
int sampleEvery;
double ratio;
int cam;

int topLeftXpix,
    topLeftYpix,
    topRightXpix,
    topRightYpix,
    bottomRightXpix,
    bottomRightYpix,
    bottomLeftXpix,
    bottomLeftYpix;
bool recording, drawing;

ros::Publisher pixelXY_pub;

int main(int argc, char **argv)
{
    /* eqn for finding distance relative to the robot
    X_meter = A * X + B , A & B are const, X is x coord of pixel
    Y_meter = C * X + D , C & D are const, Y is y coord of pixel   
    */
    // double X_meter, Y_meter;

    clock_t start;
    //timeStart;
    //timeEnd

    ros::init(argc, argv, "white_line_detection");

    ros::NodeHandle n;

    // these values are the constants
    n.param("white_line_detection_A", A, 0.0);
    n.param("white_line_detection_B", B, 0.0);
    n.param("white_line_detection_C", C, 0.0);
    n.param("white_line_detection_D", D, 0.0);
    //double A = 0.0072189, B = -2.227, C = -0.01162, D = 4.794;

    // camera select 0-builtin, 1-external
    n.param("white_line_detection_camSelect", cam, 0);
    // sampling freq
    n.param("white_line_detection_sampleEvery", sampleEvery, 4);
    // ratio
    n.param("white_line_detection_ratio", ratio, 1.3333);
    // Pixel coordinates
    n.param("white_line_detection_topLeftXpix", topLeftXpix, 247);
    n.param("white_line_detection_topLeftYpix", topLeftYpix, 218);
    n.param("white_line_detection_topRightXpix", topRightXpix, 393);
    n.param("white_line_detection_topRightYpix", topRightYpix, 216);
    n.param("white_line_detection_bottomRightXpix", bottomRightXpix, 419);
    n.param("white_line_detection_bottomRightYpix", bottomRightYpix, 290);
    n.param("white_line_detection_bottomLeftXpix", bottomLeftXpix, 224);
    n.param("white_line_detection_bottomLeftYpix", bottomLeftYpix, 295);

    // enable/disable recording
    n.param("white_line_detection_enableRecording", recording, false);

    // enable/disable drawing
    n.param("white_line_detection_enableDrawing", drawing, false);

    pixelXY_pub = n.advertise<ohm_igvc::pixel_locations>("whiteLineDistances", 5);

    namedWindow("ORIGINAL", WINDOW_AUTOSIZE);
    namedWindow("FINAL", WINDOW_AUTOSIZE);
    namedWindow("WARPED", WINDOW_AUTOSIZE);

    VideoCapture ohm_webcam(cam);

    VideoWriter recordVideo("ohm_run.avi", CV_FOURCC('M', 'J', 'P', 'G'), 10, Size(640, 480));
    ohm_webcam.set(CV_CAP_PROP_FRAME_WIDTH, 640);
    ohm_webcam.set(CV_CAP_PROP_FRAME_HEIGHT, 480);

    Mat input;          // camera feed
    Mat Cropped_region; // actual region where processing will done

    // setup for keystone correction
    Mat transmtx, transformed;
    Rect im_ROI = Rect(50, 3, 500, 381);                  // used for cropping
    Point Q1 = Point2f(topLeftXpix, topLeftYpix);         //top left pixel coordinate
    Point Q2 = Point2f(topRightXpix, topRightYpix);       //top right
    Point Q3 = Point2f(bottomRightXpix, bottomRightYpix); //bottom right
    Point Q4 = Point2f(bottomLeftXpix, bottomLeftYpix);   //bottom left

    double cardH = sqrt((Q3.x - Q2.x) * (Q3.x - Q2.x) + (Q3.y - Q2.y) * (Q3.y - Q2.y));
    double cardW = ratio * cardH;
    Rect R(Q1.x, Q1.y, cardW, cardH);

    Point R1 = Point2f(R.x, R.y);
    Point R2 = Point2f(R.x + R.width, R.y);
    Point R3 = Point2f(Point2f(R.x + R.width, R.y + R.height));
    Point R4 = Point2f(Point2f(R.x, R.y + R.height));

    std::vector<Point2f> quad_pts;
    std::vector<Point2f> squre_pts;

    quad_pts.push_back(Q1);
    quad_pts.push_back(Q2);
    quad_pts.push_back(Q3);
    quad_pts.push_back(Q4);

    squre_pts.push_back(R1);
    squre_pts.push_back(R2);
    squre_pts.push_back(R3);
    squre_pts.push_back(R4);
    transmtx = getPerspectiveTransform(quad_pts, squre_pts);
    int offsetSize = 1000;
    transformed = Mat::zeros(R.height + offsetSize, R.width + offsetSize, CV_8UC3);
    //////////////////////////////////////////////////////////////////////////////////////////////////

    //setup for white line filtering
    Mat hsv, hsv_resize, warped_resize;

    int low_H = 0, low_S = 0, low_V = 200;        // lower limit for HSV slider
    int high_H = 180, high_S = 255, high_V = 255; // upper limit for HSV slider

    createTrackbar("Low Hue", "FINAL", &low_H, 180, on_low_H_thresh_trackbar);
    createTrackbar("High Hue", "FINAL", &high_H, 180, on_high_H_thresh_trackbar);
    createTrackbar("Low Sat", "FINAL", &low_S, 255, on_low_S_thresh_trackbar);
    createTrackbar("High Sat", "FINAL", &high_S, 255, on_high_S_thresh_trackbar);
    createTrackbar("Low Val", "FINAL", &low_V, 255, on_low_V_thresh_trackbar);
    createTrackbar("High Val", "FINAL", &high_V, 255, on_high_V_thresh_trackbar);

    Point p;
    Point o;
    ////////////////////////////////////////////////////////////////////////////////////////////////
    while (ros::ok())
    {
        timeStart;
        ohm_igvc::pixel_locations msg;

        ohm_webcam >> input;
        if (input.empty())
            break;
        
        msg.time.data = ros::Time::now();
        
        // Warp -> Crop -> Change2HSV -> Threshold HSV -> Filter
        warpPerspective(input, transformed, transmtx, transformed.size());
        Cropped_region = transformed(im_ROI);

        cvtColor(Cropped_region, hsv, CV_BGR2HSV);
        inRange(hsv, Scalar(low_H, low_S, low_V, 0), Scalar(high_H, high_S, high_V, 0), hsv);

        erode(hsv, hsv, Mat(), Point(-1, -1), 3);
        dilate(hsv, hsv, Mat(), Point(-1, -1), 2);
        resize(hsv, hsv_resize, Size(100, 100), 0, 0, INTER_NEAREST); // need to recalibrate if changing size
        resize(Cropped_region, warped_resize, Size(100, 100), 0, 0, INTER_NEAREST);

        for (int i = 0; i < hsv_resize.cols; i++)
        {
            for (int j = 0; j < hsv_resize.rows; j++)
            {
                Scalar BGRv = hsv_resize.at<uchar>(j, i);
                if (BGRv.val[0] == 255)
                {

                    geometry_msgs::Point32 pixelLocation;
                    pixelLocation.x = (A * i) + B;
                    pixelLocation.y = (C * j) + D;
                    msg.pixelLocations.push_back(pixelLocation);
                    if (drawing)
                    {
                        p = Point2f(i, j);
                        o = Point2f(i + 1, j + 1);
                        rectangle(Cropped_region, p, o, Scalar(0, 255, 0), 1, 8, 0);
                    }
                }
            }
        }

        pixelXY_pub.publish(msg);

        imshow("WARPED", warped_resize);
        imshow("ORIGINAL", input);
        imshow("FINAL", hsv_resize);
        if (recording)
        {
            recordVideo.write(input);
        }
        waitKey(60);
        timeEnd;
    }
}

void on_low_H_thresh_trackbar(int, void *)
{ /*
    low_H = min(high_H - 1, low_H);
    setTrackbarPos("Low H", "Src", low_H);*/
}
void on_high_H_thresh_trackbar(int, void *)
{
    // high_H = max(high_H, low_H + 1);
    //  setTrackbarPos("High H", "Src", high_H);
}
void on_low_S_thresh_trackbar(int, void *)
{
    // low_S = min(high_S - 1, low_S);
    //  setTrackbarPos("Low S", "Src", low_S);
}
void on_high_S_thresh_trackbar(int, void *)
{
    //  high_S = max(high_S, low_S + 1);
    //  setTrackbarPos("High S", "Src", high_S);
}
void on_low_V_thresh_trackbar(int, void *)
{
    //  low_V = min(high_V - 1, low_V);
    // setTrackbarPos("Low V", "Src", low_V);
}
void on_high_V_thresh_trackbar(int, void *)
{
    //  high_V = max(high_V, low_V + 1);
    // setTrackbarPos("High V", "Src", high_V);
}
