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
//#include "NaoUtils.h"
#include "detection.h"
#include "TheWalkingNao.h"
#include "Constants.h"
#include "FeatExtract.h"
#define FOLDER  "../training_set/imgNAO/"

#define WRITE 0
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
/*	Classiwekation weka ;
	weka.ClassTest(); */
#if 0 // Estrazione Oggetti di interesse
	stringstream img_file;// = "data_set_27_05/123.jpg";
	stringstream data_file;
	Mat image; vector<Blob> blobs; vector<Mat> rectangles;
	std::ofstream writer;
	
	for(int i=1;i<127;i++){
		img_file<<FOLDER<<"im ("<<i<<").jpg";
		
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
		img_file.str(string());

		if(image.rows != HEIGH ||image.cols !=WIDTH)
			resize(image, image, Size(WIDTH,HEIGH), 0, 0, INTER_NEAREST);
	
		if (!image.data) // Check for invalid input
		{
			cout << "Could not open or find the image" << std::endl;
			return -1;
		}
		rectangles.clear();
		detect2(image,rectangles,blobs);
		
		for(int j=0;j<rectangles.size();j++){
			data_file<<FOLDER<<"detectionNoMorph/"<<"img"<<i<<"_"<<j<<".txt";
			writer.open(data_file.str(),ios::out);
			writer<<blobs[i-1].area[j]<<","<<blobs[i-1].distance[j]<<endl; //NON DIMENTICARE: i-1 perché le immagini partono da 1
			img_file<<FOLDER<<"detectionNoMorph/"<<"img"<<i<<"_"<<j<<".jpg";
			imwrite(img_file.str(),rectangles[j]);
			img_file.str("");
			data_file.str("");
			writer.clear();
			writer.close();
		}
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
#if 1
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
	dirs.push_back("detectionNoMorph/mela_rossa/");
	types.push_back("mela_rossa");
	dirs.push_back("detectionNoMorph/mela_gialla/");
	types.push_back("mela_gialla");
	dirs.push_back("detectionNoMorph/bicchiere/");
	types.push_back("bicchiere");
	dirs.push_back("detectionNoMorph/tazzina/");
	types.push_back("tazzina");

	fe.extract(dirs,"featWekaNoMorphNEW.csv",types,true);
	
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

}
