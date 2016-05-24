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
	const string img_file = "training_set/img100.jpg";
	Mat image; vector<Mat> rectangles;
	Mat interpImg = Mat::zeros(Size(1280,960), CV_8U);
	image = imread(img_file,  IMREAD_COLOR); // Read the file
	
	/*Interpolazione immagine per essere invarianti alla risoluzione*/
	resize(image, interpImg, interpImg.size(), 0, 0, INTER_LINEAR);

	if (!image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	detect(interpImg,rectangles);
	
		stringstream s;
	for(int i=0;i<rectangles.size();i++){
		s<<i;
		namedWindow(s.str(), WINDOW_AUTOSIZE);
		imshow(s.str(),rectangles[i]);
		s.str(std::string());
	}
	
	waitKey(0); // Wait for a keystroke in the window
}
