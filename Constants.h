#pragma once
#define WIDTH 640 /* Do Not Change, this is for CamParams*/
#define HEIGHT 480 /* Do Not Change, this is for CamParams*/
#define M_PI 3.14159265358979323846  /* pi */
#include "opencv2\gpu\gpu.hpp"
#include <opencv2\core\core.hpp>

//#define WIDTH 1280 /* Do Not Change, this is for CamParams*/
//#define HEIGHT 640 /* Do Not Change, this is for CamParams*/

struct Blob {
	double area;
	cv::Point centroid;
	double distance;
	cv::Mat cuttedImages;
	cv::Mat* originalImage;
	cv::Mat blobsImage;
	cv::Mat cuttedWithBack;
	cv::Rect rectangles;
};

