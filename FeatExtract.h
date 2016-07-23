#pragma once

#include <iostream>

#include "dirent.h"
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream> 
#include "histogram.hpp"
#include "lbp.hpp"
#include "Constants.h"

#define HEADER "meanHue, mom1, mom2, mom3, mom4, mom5, mom6, mom7,stdDevHue,stdDevSaturation, stdDevValue, entropy,ratio,W,R,Y,meanB,meanG,meanR,area,distance,"

using namespace std;
using namespace cv;

#define BGR_FEAT_COUNT 3
#define LBP_FEAT_COUNT 10
#define HEADER_BGR "B,G,R,"
#define HEADER_LBP "0,1,16,224,225,239,240,241,248,254,"

class FeatExtract
{
	
private:
	double Log2( double n ) ;
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
        std::string FeatExtract::extractDuringMovement(Blob blob,  bool);
	std::string readMeanHueAndMoments(cv::Mat image);
	std::string readStdDevHSV(cv::Mat image);
	double computeEntropy(cv::Mat );
    double computeRectangleRatio(cv::Mat image); //In fase di training
	double computeRectangleRatio(cv::Rect r) ;	 //In faseoperativa
	std::string computeColorFeatures(cv::Mat image);
	//double readBboxComparasion(cv::Mat image);	

        /* estrazione grezza delle feature: uso interno */
        void extractBGRFeatures(Mat& bgrImage, double bgrFeatures[]);
        void extractLBPFeatures(Mat& bgrImage, double lbpFeatures[]);

        /* estrazione stringa di feature per i due tipi di classificatore */
        std::string extractForColorClassifier(Blob blob, bool toMask);
        std::string extractForLBPClassifier(Blob blob, bool toMask);
};

