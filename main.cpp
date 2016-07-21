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
#define FOLDER  "../training_set/imgNAO/"

#define WRITE 0
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
#if 0  //CLASSIFICAZIONE
	Classiwekation weka ;
	stringstream img_file;// = "data_set_27_05/123.jpg";
	stringstream feat;
	Mat image, toShow; vector<Blob> blob; vector<Mat> rectangles;
	FeatExtract fe;

	for(int i=20;i<124;i++){
		img_file<<FOLDER<<"im ("<<i<<").jpg";
		
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());

	/*	if(image.rows != HEIGH ||image.cols !=WIDTH)
			resize(image, image, Size(WIDTH,HEIGH), 0, 0, INTER_NEAREST);*/
	
		if (!image.data) // Check for invalid input
		{	cout << "Could not open or find the image" << std::endl;
			return -1;}
		
		rectangles.clear();
		detect2(image,rectangles,blob);
		
		for(int j=0;j<blob.size();j++){
				std::string winner;
			/*Qui eventuale codice sui rettangoli o sui blob*/

			feat<<fe.extractDuringMovement(blob[j],false);
			cout<<feat.str()<<endl;
			double result = weka.classify(feat.str());
			feat.str("");
			if(result==1)
				winner = "mela rossa";
			else if (result==2)
				winner = "mela gialla";
			else if(result==3)
				winner = "bicchiere";
			else if(result==4)
				winner = "tazzina";
			else if(result==5)
				winner ="nessuno";
			rectangle(image,blob[j].rectangles.tl(),blob[j].rectangles.br(),Scalar(0,0,255));
			putText(image,winner,blob[j].rectangles.tl(),FONT_HERSHEY_PLAIN, 1.0, CV_RGB(0, 255, 0), 2.0);
			result = 0;


			/*data_file<<FOLDER<<"detectionNoMorph/"<<"img"<<i<<"_"<<j<<".txt";
			writer.open(data_file.str(),ios::out);
			writer<<blobs[i-1].area[j]<<","<<blobs[i-1].distance[j]<<endl; //NON DIMENTICARE: i-1 perché le immagini partono da 1
			img_file<<FOLDER<<"detectionNoMorph/"<<"img"<<i<<"_"<<j<<".jpg";
			imwrite(img_file.str(),rectangles[j]);
			img_file.str("");
			data_file.str("");
			writer.clear();
			writer.close();*/
		}
		blob.clear();
		imshow("out",image);
		image = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
		waitKey(0);
		
	}

		
#endif
#if 0 // Estrazione Oggetti di interesse
	stringstream img_file;// = "data_set_27_05/123.jpg";
	stringstream data_file;
	Mat image;vector<Blob> blob; vector<Mat> rectangles;
	std::ofstream writer;
	
	for(int i=21;i<181;i++){
		img_file<<FOLDER<<"im ("<<i<<").jpg";
		
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());

	 	if(image.rows != 480 ||image.cols !=640)
			resize(image, image, Size(640,480), 0, 0, INTER_NEAREST);
	
		if (!image.data) // Check for invalid input
		{
			cout << "Could not open or find the image" << std::endl;
			return -1;
		}
		rectangles.clear();
		detect2(image,rectangles,blob);
		
		for(int j=0;j<blob.size();j++){
			/*Qui eventuale codice sui rettangoli o sui blob*/
			
			data_file<<FOLDER<<"detection/"<<"img"<<i<<"_"<<j<<".txt";
			writer.open(data_file.str(),ios::out);
			writer<<blob[j].area<<","<<blob[j].distance<<endl; //NON DIMENTICARE: i-1 perché le immagini partono da 1
			img_file<<FOLDER<<"detection/"<<"img"<<i<<"_"<<j<<".jpg";
			imwrite(img_file.str(),blob[j].cuttedWithBack);
			img_file.str("");
			data_file.str("");
			writer.clear();
			writer.close();
		}
		blob.clear();
		image = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	}

	waitKey(0);

#endif
#if 0
	stringstream img_file; img_file<< "data_set_27_05/47.jpg";
	Mat image; vector<Mat> rectangles;
	image = imread(img_file.str(),  IMREAD_COLOR)+1;

	if(image.rows < 960 ||image.cols <1280)
			resize(image, image, Size(1280,960), 0, 0, INTER_LINEAR);
	
		if (!image.data) // Check for invalid input
		{
			cout << "Could not open or find the image" << std::endl;
			return -1;
		}
		detect2(image,rectangles);
		stringstream s;
	for(int i=0;i<rectangles.size();i++){
		s<<i;
		namedWindow(s.str(), WINDOW_AUTOSIZE);
		imshow(s.str(),rectangles[i]);
		s.str(std::string());
	}
	waitKey(0); // Wait for a keystroke in the window
#endif
#if 0 //Estrazione Features
	FeatExtract fe;
	vector<string> dirs,types;
/*	dirs.push_back("../detection/mela_rossa/");
	types.push_back("mela_rossa");
	dirs.push_back("../detection/mela_gialla/");
	types.push_back("mela_gialla");
	dirs.push_back("../detection/bicchiere/");
	types.push_back("bicchiere");
	dirs.push_back("../detection/tazzina/");
	types.push_back("tazzina");

	fe.extract(dirs,"featWeka.csv",types,false);
	*/
	dirs.push_back("../training/mela_rossa/");
	types.push_back("mela_rossa");
	dirs.push_back("../training/mela_gialla/");
	types.push_back("mela_gialla");
	dirs.push_back("../training/bicchiere/");
	types.push_back("bicchiere");
	dirs.push_back("../training/tazzina/");
	types.push_back("tazzina");
	dirs.push_back("../training/nao/");
	types.push_back("nao");


	fe.extract(dirs,"featWeka17_07Refit.csv",types,false);
	
	waitKey(0);
	system("pause");

#endif
#if 0
	NaoUtils n;
	n.explore();
	system("pause");
#endif
#if 0
	TheWalkingNao twn;
	stringstream img_file;// = "data_set_27_05/123.jpg";
	Mat image; vector<Mat> rectangles; double angle=-1;
	Mat sharp;
	for(int i=1;i<64;i++){
		img_file<<"Markers\\invert\\im ("<<i<<").jpg";
		cout<<img_file.str()<<endl;
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());

		if(image.rows < 640 ||image.cols <480)
			resize(image, image, Size(640,480), 0, 0, INTER_LINEAR);
		twn.ArucoFind(image,angle,false);
		cout<<"Angolo" <<angle<<endl;
		imshow("img",image);
		waitKey(0);
	
	}
#endif

/*DA QUI IN POI CODIFICHIAMO IL COMPORTAMENTO DI NAO*/
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
		}catch(std::exception& e){cout<<e.what()<<endl;twn.restNow();		}
	}
	twn.restNow();
#endif
#if 0
	NaoUtils nu; TheWalkingNao twn;
//	twn.walk(0.0,0.0);
	nu.takeSomePhotos("calib/");
#endif
	SimpleBlobDetector::Params params;
	params.minThreshold = 150;
	params.maxThreshold = 220;
	params.thresholdStep = 2;
	params.filterByArea =false;
	params.maxArea = 10000;
	params.minArea = 500 ;
	params.filterByInertia = false;
	params.maxInertiaRatio = 0.5;
	SimpleBlobDetector detector(params);

 
// Draw detected blobs as red circles.
// DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob

	Mat image;
	stringstream img_file;
	for(int i=1;i<64;i++){
			std::vector<KeyPoint> keypoints;
			
		img_file<<"blobs\\im ("<<i<<").jpg";
		cout<<img_file.str()<<endl;
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());
		detector.detect( image, keypoints);
		Mat im_with_keypoints;
		drawKeypoints( image, keypoints, im_with_keypoints, Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
		imshow("blobs",im_with_keypoints);
		waitKey(0);

	}
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
