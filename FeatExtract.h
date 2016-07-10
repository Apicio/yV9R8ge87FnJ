#pragma once

#include <iostream>
#include "dirent.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream> 
#define HEADER "BBoxComp,meanHue, mom1, mom2, mom3, mom4, mom5, mom6, mom7,stdDevHue,"

using namespace std;

class FeatExtract
{
public:
	std::ofstream writer;
	FeatExtract(void);
	~FeatExtract(void);
	/**
	* @brief Estrae un insieme di features. NOTA BENE: aggiungere alla costante HEADER eventuali nuove features
	* @param pathToDir Cartella in cui sono presenti le immagini
	* @param pathToFile file in cui salvare le features 
	* @param type nome della classe 
	* @return The class label, or 0 for rejection
	*/
	void extract(std::vector<string> pathToDir, std::string pathToFile, std::vector<string> types,bool toMask);
	std::string FeatExtract::extractDuringMovement(cv::Mat img,  bool toMask);
	std::string readMeanHueAndMoments(cv::Mat image);
	std::string readStdDevHue(cv::Mat image);
	double readBboxComparasion(cv::Mat image);
	
};

