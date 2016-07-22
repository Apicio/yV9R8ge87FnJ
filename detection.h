#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2/opencv.hpp>
#include <vector>
#include "Constants.h"
#include "BlobResult.h"
#include "opencv2\gpu\gpu.hpp"

#define HEIGH 480
#define WIDTH 640
#define MAX_AREA 10000
#define MIN_AREA 400
#define RECT_AUGMENT 0.2
#define SHADOW_THRESH 80
#define MEAN_OFFSET
#define DO_MORPH 0

using namespace std;
using namespace cv;

/**
 * @brief Detect regions of interest in an image
 * @param image Raw image to analize
 * @param regionsOfInterest Array of ROIs detected
 */
void detect(Mat image, vector<Mat>& regionsOfInterest);
void detect2(Mat , vector<Mat>&, vector<Blob>&);
void detect3(Mat , vector<Mat>&,Blob&);
vector<double> computeArea(vector<vector<Point> > contours);
Mat backgroundRemoval(Mat& img);
Mat applyMaskBandByBand(Mat mask, vector<Mat> bands);
Mat sharpening(Mat in, double sigma);
void imadjust(const Mat1b& src, Mat1b& dst, int tol, Vec2i in, Vec2i out);
Mat computeRationedImage(vector<Mat> bands);
double getThreshVal_Otsu_8u( const cv::Mat& _src );
Rect refitToBorders(Mat region);
Rect resizeRectangle(Rect r);
void rgb2cmyk(cv::Mat& img, std::vector<cv::Mat>& cmyk) ;

#endif // DETECTION_H
