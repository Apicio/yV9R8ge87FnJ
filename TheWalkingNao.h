#pragma once
#include <stdio.h>
#include <jni.h>
#include <string.h>
#include "aruco.h"
#include "cvdrawingutils.h"
#include <iostream>
#include <opencv2/opencv.hpp>
#include <vector>
#include <fstream> 
#include <math.h>

#include <opencv2/highgui/highgui.hpp>
#define SCALE1 1.00661601728129 /* Il marker non è quadrato */
#define SCALE2 1.00660122390337
#define SCALE3 1.1147142716032
#define SCALE4 1.11475005182036
#define SCALE5 1.10453569751983
#define SCALE6 1.10740487910035
#define ANGLE_PHASE 0.785398 /* Tracciamo un'angolo, serve aggiungere una fase DA CONTROLLARE*/
#define M_PI 3.14159265358979323846  /* pi */

using namespace cv;
using namespace aruco;

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

int pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}

double getOrientation(Marker marker){
	Point cent(0, 0); // Calcolo il centroide
    for (int i = 0; i < 4; i++) {
        cent.x += marker.at(i).x;
        cent.y += marker.at(i).y;
    }
    cent.x /= 4.;
    cent.y /= 4.;
	double scaleFactor = 1;
 	double x[] = {SCALE1*(marker.at(0).x-cent.x), marker.at(1).x-cent.x, marker.at(2).x-cent.x, SCALE2*(marker.at(3).x-cent.x)};
	double y[] = {SCALE3*(marker.at(0).y-cent.y), SCALE4*(marker.at(1).y-cent.y), SCALE5*(marker.at(2).y-cent.y), SCALE6*(marker.at(3).y-cent.y)};
	double d1 = sqrt(pow(abs(x[1]-x[2]),2)+pow(abs(y[1]-y[2]),2)); // Ordine Punti R0, G1, B2
	double d2 = sqrt(pow(abs(x[1]-x[0]),2)+pow(abs(y[1]-y[0]),2));
	double normX = x[1]*d1/d2;
	double normY = y[1]*d2/d1;
	cout << normY << " " <<  normX;
	return 180*(atan2(normY,normX)+ANGLE_PHASE)/M_PI;
}

class TheWalkingNao
{
private:

public:
	TheWalkingNao(void);
	void ArucoFind(void);
	~TheWalkingNao(void);
};