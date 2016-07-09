#pragma once
#include <stdio.h>
#include <string.h>
#include <iostream>
#include <vector>
#include <fstream> 
#include <math.h>

#include "aruco.h"
#include "cvdrawingutils.h"
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#define M_PI 3.14159265358979323846  /* pi */

using namespace cv;
using namespace aruco;

/*
double fmin(double element[], int size){
	double small = element[0];
	for(int i=0; i<size; i++){
		if (element[i] < small)
			small = element[i];
	}
	return small;
}
double fmax(double element[], int size){
	double small = element[0];
	for(int i=0; i<size; i++){
		if (element[i] > small)
			small = element[i];
	}
	return small;
}
*/
enum Direction {NORTH,SOUTH,WEST,OVEST, NORTHWEST,SOUTHWEST,SOUTHOVEST,NORDOVEST };
class TheWalkingNao
{
private:
	int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
	double computeAngle(Marker m, CameraParameters cam);
	double fmin(double element[], int size);
	double fmax(double element[], int size);
public:
	TheWalkingNao(void);
	void ArucoFind(Mat img, double& angle,bool toRemoveMarkers);
	~TheWalkingNao(void);
};