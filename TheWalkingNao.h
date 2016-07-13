#pragma once
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include "cvdrawingutils.h"
#include "detection.h"
#include "Constants.h"
#include <string.h>
#include <iostream>
#include <limits.h>
#include <fstream> 
#include <stdio.h>
#include "aruco.h"
#include <vector>
#include <math.h>

using namespace cv;
using namespace aruco;

enum Direction {NORTH,SOUTH,WEST,OVEST, NORTHWEST,SOUTHWEST,SOUTHOVEST,NORDOVEST };
class TheWalkingNao
{
private:
	CameraParameters camParams;
	bool _ImageSharp;
	bool _invert;
	double _SharpSigma;
	double _SharpThreshold;
	double _SharpAmount;
	double _medianBlur;
	double _markSize; /* m */


	int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
	double computeAngle(Marker m, CameraParameters cam);
	double fmin(double element[], int size);
	double fmax(double element[], int size);
public:
	TheWalkingNao(void);
	void ArucoFind(Mat img, double& angle,bool toRemoveMarkers);
	~TheWalkingNao(void);
};