#ifndef DETECTION_H
#define DETECTION_H

#include <opencv2\opencv.hpp>
#include <opencv2\core\core.hpp>
#include <opencv2\highgui\highgui.hpp>
#include <vector>

using namespace std;
using namespace cv;

/**
 * @brief Detect regions of interest in an image
 * @param image Raw image to analize
 * @param regionsOfInterest Array of ROIs detected
 */
void detect(Mat image, vector<Rect>& regionsOfInterest);

#endif // DETECTION_H
