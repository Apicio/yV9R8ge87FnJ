/**
 * Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved
 * \file sayhelloworld.cpp
 * \brief Make NAO say a short phrase.
 *
 * A simple example showing how to make NAO say a short phrase using the
 * specialized proxy ALTextToSpeechProxy.
 */

#include <iostream>

#include <iostream>
#include "NaoUtils.h"
#include "detection.h"


#include "FeatExtract.h"
#define FOLDER  "data_set_27_05/"

#define WRITE 0
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
/*	Classiwekation weka ;
	weka.ClassTest(); */
#if 0
	stringstream img_file;// = "data_set_27_05/123.jpg";
	Mat image; vector<Mat> rectangles;

	for(int i=0;i<156;i++){
		img_file<<FOLDER<<i<<".jpg";
		image = imread(img_file.str(),  IMREAD_COLOR)+1; // Read the file. +1 perch� nel rationing non vogliamo dividere per 0!
		img_file.str(string());

		if(image.rows < 960 ||image.cols <1280)
			resize(image, image, Size(1280,960), 0, 0, INTER_LINEAR);
	
		if (!image.data) // Check for invalid input
		{
			cout << "Could not open or find the image" << std::endl;
			return -1;
		}
		rectangles.clear();
		detect2(image,rectangles);
		
		for(int j=0;j<rectangles.size();j++){
			img_file<<FOLDER<<"detectionNoMorph/"<<"img"<<i<<"_"<<j<<".jpg";
			imwrite(img_file.str(),rectangles[j]);
			img_file.str(string());
		}
		image = Mat::zeros(Size(1280,960), CV_8U);
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
#if 0
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
	dirs.push_back("../detectionNoMorph/mela_rossa/");
	types.push_back("mela_rossa");
	dirs.push_back("../detectionNoMorph/mela_gialla/");
	types.push_back("mela_gialla");
	dirs.push_back("../detectionNoMorph/bicchiere/");
	types.push_back("bicchiere");
	dirs.push_back("../detectionNoMorph/tazzina/");
	types.push_back("tazzina");

	fe.extract(dirs,"featWekaNoMorph.csv",types,true);
	
	waitKey(0);
	system("pause");

#endif
	NaoUtils n;
	n.explore();
	system("pause");
}
