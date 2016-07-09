#pragma once
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream> 
#include <math.h>

#include "ArucoSrc\aruco.h"
#include "ArucoSrc\cvdrawingutils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define M_PI 3.14159265358979323846  /* pi */

using namespace cv;
using namespace aruco;




class TheWalkingNao
{
private:
	int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
	double computeAngle(Marker m, CameraParameters cam);
public:
	TheWalkingNao(void);
	void ArucoFind(Mat img, double& angle);
	~TheWalkingNao(void);
};