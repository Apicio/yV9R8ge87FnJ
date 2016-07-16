#pragma once
#define WIDTH 640 /* Do Not Change, this is for CamParams*/
#define HEIGHT 480 /* Do Not Change, this is for CamParams*/
#define M_PI 3.14159265358979323846  /* pi */

//#define WIDTH 1280 /* Do Not Change, this is for CamParams*/
//#define HEIGHT 640 /* Do Not Change, this is for CamParams*/

struct Blob {
	std::vector<double> area;
	std::vector<cv::Point> centroid;
	std::vector<double> distance;
	std::vector<cv::Mat> cuttedImages;
	cv::Mat originalImage;
	std::vector<cv::Mat> blobsImage;
	std::vector<cv::Rect> rectangles;
};

