#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>

#define MAX_AREA 40000
#define MIN_AREA 1000
#define RECT_AUGMENT 0.07
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
void detect2(Mat image, vector<Mat>& regionsOfInterest);
vector<double> computeArea(vector<vector<Point> > contours);

#endif // DETECTION_H
