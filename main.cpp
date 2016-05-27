/**
 * Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved
 * \file sayhelloworld.cpp
 * \brief Make NAO say a short phrase.
 *
 * A simple example showing how to make NAO say a short phrase using the
 * specialized proxy ALTextToSpeechProxy.
 */

#include <iostream>
#include <alerror\alerror.h>
#include <alproxies\altexttospeechproxy.h>
#include <iostream>

#include "detection.h"
using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	const string img_file = "training_set/img194.jpg";
	Mat image; vector<Mat> rectangles;

	image = imread(img_file,  IMREAD_COLOR)+1; // Read the file. +1 perché nel rationing non vogliamo dividere per 0!
	
	//Mat interpImg = Mat::zeros(Size(1280,960), image.type());
	/*Interpolazione immagine per essere invarianti alla risoluzione*/
	if(image.rows < 960 ||image.cols <1280)
		resize(image, image, Size(1280,960), 0, 0, INTER_LINEAR);
	
	if (!image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	detect(image,rectangles);
	
		stringstream s;
	for(int i=0;i<rectangles.size();i++){
		s<<i;
		namedWindow(s.str(), WINDOW_AUTOSIZE);
		imshow(s.str(),rectangles[i]);
		s.str(std::string());
	}
	
	waitKey(0); // Wait for a keystroke in the window
}
