#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

#define HEIGH 480
#define WIDTH 640
#define MAX_AREA 40000
#define MIN_AREA 5000
#define RECT_AUGMENT 0.1
#define SHADOW_THRESH 126
#define MEAN_OFFSET

using namespace std;
using namespace cv;

/**
 * @brief Detect regions of interest in an image
 * @param image Raw image to analize
 * @param regionsOfInterest Array of ROIs detected
 */
void detect(Mat image, vector<Mat>& regionsOfInterest);
void detect2(Mat , vector<Mat>&, vector<double>&,vector<double>& );
vector<double> computeArea(vector<vector<Point> > contours);
Mat backgroundRemoval(Mat& img);
Mat applyMaskBandByBand(Mat mask, vector<Mat> bands);
Mat sharpening(Mat in, double sigma);

#endif // DETECTION_H
