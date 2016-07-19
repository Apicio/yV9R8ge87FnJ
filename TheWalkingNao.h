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
// Aldebaran includes.
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <opencv2\gpu\gpu.hpp>

#define PORT 9559

using namespace cv;
using namespace aruco;
using namespace AL;
using namespace gpu;

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

	
	ALMotionProxy* motion;
	ALRobotPostureProxy* robotPosture;
	
	int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
	double computeAngle(Marker m, CameraParameters cam);
	double fmin(double element[], int size);
	double fmax(double element[], int size);
public:
	
	TheWalkingNao();
	void ArucoFind(Mat img, double& angle,bool toRemoveMarkers);
	void standUp();
	void moveLeft(float meters,double angle);
	void moveRight(float meters, double angle);
	void moveForward(float meters);
	void restNow();
	bool isMoving();
	void init(const char* robotIP);
	~TheWalkingNao(void);
};

