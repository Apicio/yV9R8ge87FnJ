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
#include "NaoUtils.h"
// Aldebaran includes.
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <opencv2\gpu\gpu.hpp>
#define WSPEED 0.2 /* m/s  */
#define RSPEED 0.1 /* rad/s */
#define WDIST 0.2 /* meters */

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
	vector<Marker> ArucoFind(Mat img, double& angle,bool toRemoveMarkers);
	void infiniteRotate(float velTheta);
	void moveNearMarker(Mat& img, NaoUtils nu, ALVideoDeviceProxy camProx);
	void standUp();
	void moveLeft(float meters,double angle);
	void moveRight(float meters, double angle);
	void moveForward(float meters);
	void rotate(float angle);
	void restNow();
	bool isMoving();
	void moveOnX(float meters);
	void moveUpNeck();
	void moveDownNeck(float PitchAngle);
	void walk(float X, float Y);
	void infiniteWalk(float velX, float velY, float);
	void init(const char* robotIP);
	~TheWalkingNao(void);
};

