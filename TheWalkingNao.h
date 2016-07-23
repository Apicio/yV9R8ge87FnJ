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
#include "BlobResult.h"
#include "blob.h"
// Aldebaran includes.
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <opencv2\gpu\gpu.hpp>
#include <alproxies/altexttospeechproxy.h>

#define RESIZE_COEFF 3

#define PORT 9559

using namespace cv;
using namespace aruco;
using namespace AL;
using namespace gpu;

enum Direction {UP, RIGHT, DOWN, LEFT, STOP};
extern char* s_Direction[];
struct MarkersInfo{
	Size img_size;
	Direction dir;
	Point center;
	Mat marker;
	CvRect rect;
	std::vector<cv::Point> curve;
	bool isValid;
	float angle;
	vector<Point> contour;
	MarkersInfo(){
		isValid = false;
	}
};
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
	ALValue _imger;
/*	ALTextToSpeechProxy* tts;
	ALMemoryProxy* memoryProxy;
	const char* ip;
	float initial_gyro;*/
	
	int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy);
	double computeAngle(Marker m, CameraParameters cam);
	double fmin(double element[], int size);
	double fmax(double element[], int size);
public:
	
	TheWalkingNao();
	vector<Marker> ArucoFind(Mat img, double& angle,bool toRemoveMarkers);
	void saySomething(cv::string);
	void infiniteRotate(float velTheta);
	void moveNearMarker(NaoUtils nu, ALVideoDeviceProxy camProx);
	void standUp();
	void standCrouch();
	void moveLeft(float meters,double angle);
	void moveRight(float meters, double angle);
	void moveForward(float meters);
	void rotate(float angle);
	void restNow();
	bool isMoving();
	void moveOnX(float meters);
	void moveUpNeck();
	bool pathfinder(cv::Mat orig, Direction&, Point&, float& angle, Mat&); // AGGIUNGERE VALORI DI DEFAULT
	void moveDownNeck(float PitchAngle);
	void walk(float X, float Y, float angle);
	void markerExplore(ALVideoDeviceProxy, NaoUtils );
	vector<Mat>  objectsExplore(ALVideoDeviceProxy, NaoUtils );
	void infiniteWalk(float velX, float velY, float);
	void init(const char* robotIP);
	bool rotateAllign(ALVideoDeviceProxy camProx, NaoUtils nu, Size img_size, Point p, float angle);
	vector<Point> getTwoNearY(vector<Point> points);
	float calculateAngle(vector<Point> points);
	float tryGyroscope();
	~TheWalkingNao(void);
};

