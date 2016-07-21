/**
 * Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved
 * \file sayhelloworld.cpp
 * \brief Make NAO say a short phrase.
 *
 * A simple example showing how to make NAO say a short phrase using the
 * specialized proxy ALTextToSpeechProxy.
 */

#include <iostream>
#include <fstream>
#include "NaoUtils.h"
#include "detection.h"
#include "TheWalkingNao.h"
#include "Classiwekation.h"
#include "Constants.h"
#include "FeatExtract.h"
#include "slic.h"
#include "BlobResult.h"
#include "blob.h"
#define FOLDER  "../DatasetLippi"

#define WRITE 0
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
#if 0
	Classiwekation weka ;
	double result = weka.classify("0.046135, 0.00243138,4.845e-008,1.22376e-010,3.1175e-010,3.34319e-020,-6.68581e-014,5.08932e-020,43.8282,4.54863,21177,396,?");
	cout << "Class =" << result << endl;
	result = weka.classify("0.0511247, 0.000900998,1.61258e-009,4.42675e-013,2.07049e-012,1.88707e-024,-3.97962e-017,-6.06773e-025,16.2016,5.16613,26625.5,309,?");
	cout << "Class =" << result << endl;
	result = weka.classify("0.0389009, 0.00130581,1.40611e-007,1.74627e-010,6.37281e-010,2.11688e-019,2.34569e-013,-1.96148e-020,33.268,1.68026,18561.5,623,?");
	cout << "Class =" << result << endl;
	result = weka.classify("0.0371679, 0.0010112,1.75163e-007,8.84218e-011,2.84371e-011,1.32563e-021,1.19016e-014,5.25427e-022,35.3591,2.84863,10448,509,?");
	cout << "Class =" << result << endl;
#endif
	#if 0
 TheWalkingNao twn; char* ip = "192.168.88.202";
 NaoUtils nu; ALVideoDeviceProxy camProx(ip, NAOPORT);
 twn.init(ip); vector<Mat> buff;
 int keyPressed = 0;
 twn.standUp(); 
 double angle=0;;
 while(cv::waitKey(1) != 'e'){
  Mat image;

  try{
  image = nu.see(camProx);
  //buff.push_back(image);
  twn.moveNearMarker(image, nu, camProx);
  /*if(angle>0 && angle <90 && !twn.isMoving())
   twn.moveRight(0.2,angle);
  else if(angle >90 && angle <180 && !twn.isMoving())
   twn.moveLeft(0.2,angle);
  else if(!twn.isMoving()){ twn.moveForward(0.1); angle = 0;}*/
  }catch(std::exception& e){cout<<e.what()<<endl;twn.restNow();  }
 }
 twn.restNow();
#endif
#if 0
	Mat image; 
	for(int i=1;i<121;i++){
		stringstream img_file;
		vector<Mat> rectangles;
		vector<Blob> blob;
		img_file<<"DATASET_18_07\\im ("<<i<<").jpg"; // 12 28 42
		cout << img_file.str() << endl;
		image = imread(img_file.str(),  IMREAD_COLOR);
		resize(image, image, Size(WIDTH,HEIGH), 0, 0, INTER_NEAREST);
		detect2(image,rectangles, blob);

/*		stringstream s;
	for(int i=0;i<rectangles.size();i++){
		s<<i;
		namedWindow(s.str(), WINDOW_AUTOSIZE);
		imshow(s.str(),rectangles[i]);
		s.str(std::string());
	}*/
		imshow("anc",image);
/*		for(int k=0; k<blob.cuttedImages.size(); k++){
			stringstream s;
			s << "full" << k;
			imshow(s.str(),blob.cuttedImages.at(k));
			waitKey(1000); /// Wait for a keystroke in the window
		}*/
	}
#endif
#if 1
	TheWalkingNao twn;
	stringstream img_file;// = "data_set_27_05/123.jpg";
	Mat image;
	Mat sharp;
	for(int i=1;i<4;i++){
		img_file<<"Markers\\New\\chiat ("<<i<<").jpg";
		cout<<img_file.str()<<endl;
		image = imread(img_file.str(),  IMREAD_COLOR); // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());
		Mat t = twn.pathfinder(image);
		imshow("anv",t);
		waitKey(1);
		//imshow("img",image);
		//waitKey(0);
	}
#endif
#if 0
	TheWalkingNao twn;
	stringstream img_file;// = "data_set_27_05/123.jpg";
	Mat image; vector<Mat> rectangles; double angle=-1;
	Mat sharp;
	for(int i=26;i<29;i++){
		img_file<<"Markers\\Invert\\im ("<<i<<").jpg";
		cout<<img_file.str()<<endl;
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());
		twn.ArucoFind(image,angle,false);
		cout<<"Angolo" <<angle<<endl;
		//imshow("img",image);
		//waitKey(0);
	}
#endif
#if 0
	cv::Mat img, result;
for(int i=1;i<121;i++){
		stringstream img_file;
		img_file<<"imgNao\\im ("<<i<<").jpg";
		img = imread(img_file.str());
	int numSuperpixel = 200;

	SLIC slic;
	slic.GenerateSuperpixels(img, numSuperpixel);
	if (img.channels() == 3) 
		result = slic.GetImgWithContours(cv::Scalar(0, 0, 255));
	else
		result = slic.GetImgWithContours(cv::Scalar(128));

	imshow("slic",result);
	waitKey(1);
}
#endif
#if 0
	cv::Mat img;
for(int i=1;i<121;i++){
		stringstream img_file;
		img_file<<"DatasetLippi\\im ("<<i<<").jpg";
		img = imread(img_file.str());
int width = img.cols;
int height = img.rows;
resize(img, img, Size(width/1.5,height/1.5), 0, 0, INTER_NEAREST);
width = img.cols;
height = img.rows;
int sz = width*height;
int m_compactness = 5;
int m_spcount = 200;
int numlabels = 5;
int* labels = new int[sz];
SLIC slic;
//---------------------------------------------------------
if(m_spcount < 20 || m_spcount > sz/4) m_spcount = sz/200;//i.e the default size of the superpixel is 200 pixels
if(m_compactness < 1.0 || m_compactness > 80.0) m_compactness = 20.0;
//---------------------------------------------------------

UINT* imgBuffer = new UINT[sz];
cv::Mat newImage;
cv::cvtColor(img, newImage, CV_BGR2BGRA);
memcpy( imgBuffer, (UINT*)newImage.data, sz*sizeof(UINT) );

slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(imgBuffer, width, height, labels, numlabels, m_spcount, m_compactness);
//slic.DoSuperpixelSegmentation_ForGivenSuperpixelSize(img, width, height, labels, numlabels, 10, m_compactness);//demo
slic.DrawContoursAroundSegments(imgBuffer, labels, width, height, Scalar(0,0,0));
vector<SuperPixel > set;
bool** matrix = NULL;
slic.GetPixelsSet(img, labels, width, height, numlabels, set, matrix);

#if 0
for(int i=0; i<set.size(); i++){
	cv::Mat rr = cv::Mat::zeros(height,width,CV_8U);
	vector<cv::Point > component = set.at(i).points;
	Mat superpixel = cv::Mat::zeros(1,component.size(),CV_8UC3);
	int idx = 0;
	for(int j=0; j<component.size(); j++){
		cv::Point p = component.at(j);
			rr.at<uchar>(p.y,p.x) = 255;
			superpixel.at<Vec3b>(0,idx) = img.at<Vec3b>(p.y,p.x);
			idx++;
	}
	for(int kk = 0; kk<numlabels; kk++){
		if(matrix[i][kk] || matrix[kk][i]){
			vector<cv::Point > component = set.at(kk).points;
			int value = rand() %175+80;
			for(int j=0; j<component.size(); j++){
				cv::Point p = component.at(j);
				rr.at<uchar>(p.y,p.x) = value;			
			}
		}
	}
	cv::imshow("ewwiwa",rr);
	cv::waitKey(300);
}

//if(labels) delete [] labels;
cv::Mat result(height, width, CV_8UC4);
memcpy(result.data, imgBuffer, sz*sizeof(UINT));
cvtColor(result, result, CV_BGRA2BGR);
//picHand.SavePicture(imgBuffer, width, height, picvec[k], saveLocation, 1, "_SLIC");// 0 is for BMP and 1 for JPEG)
//if(img) delete [] img;
imshow("abc",result);
waitKey(1);

Mat bands[3];
split(result,bands);
Mat mask1 = bands[0] == 0;
Mat mask2 = bands[1] == 0;
Mat mask3 = bands[2] == 0;
Mat mask4 = Mat::zeros(img.size(),CV_8U);
for(int idx = 0; idx < mask4.rows; idx++){
	mask4.at<uchar>(idx,0) = 255;
	mask4.at<uchar>(idx,1) = 255;
	mask4.at<uchar>(idx,mask4.cols-1) = 255;
	mask4.at<uchar>(idx,mask4.cols-2) = 255;
}
for(int idx = 0; idx < mask4.cols; idx++){
	mask4.at<uchar>(0,idx) = 255;
	mask4.at<uchar>(1,idx) = 255;
	mask4.at<uchar>(mask4.rows-1,idx) = 255;
	mask4.at<uchar>(mask4.rows-2,idx) = 255;
}
bitwise_and(mask1,  mask2, mask2);
bitwise_and(mask3,  mask2, mask3);
mask4 = mask3 + mask4;
imshow("abcd",mask4);
waitKey(1);
vector< vector<Point> > contours;
findContours(mask4, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
Mat toReturn = img.clone();
for(int uu = 0; uu<contours.size(); uu++){
	drawContours(toReturn,contours,uu,Scalar(rand() % 256,rand() % 256,rand() % 256),CV_FILLED);
}

imshow("abcde",toReturn);
waitKey(1);
cout << i << endl;
#endif
}

#endif



/*
whilte non siamo a fine percorso
leggi immagine della camera

if nao non è seduto
	ottieni informazioni di percorso usando aruco

if marker
	muoviti verso il marker in modo da starci completamente sopra così che non copaia nella camera
else
	continua con l'ultima direzione valida

if marker di direzione
	esplra in accordo alla direzione

if marker di stop
	siediti sul marker, nao si siede rivolto nella direzione di marcia

if nao è fermo
	esplora ruotando la testa
	rileva oggetto
	pronuncia nome oggetto

if rilevato o time out
	allinea la testa nella direzione di marcia
	alzati
*/
	system("pause");
}
