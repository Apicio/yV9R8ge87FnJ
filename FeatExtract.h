#pragma once

#include <iostream>
#include "dirent.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream> 
#define HEADER "meanHue, mom1, mom2, mom3, mom4, mom5, mom6, mom7,"

using namespace std;

class FeatExtract
{
public:
	std::ofstream writer;
	FeatExtract(void);
	~FeatExtract(void);
	void extract(std::string pathToDir, std::string pathToFile, std::string type);
	std::string readMeanHueAndMoments(cv::Mat image);
};

