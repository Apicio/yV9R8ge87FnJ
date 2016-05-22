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
	const string img_file = "immagini/img26.jpg";
	Mat image; vector<Rect> rectangles;
	image = imread(img_file,  IMREAD_COLOR); // Read the file

	if (!image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	detect(image,rectangles);

	waitKey(0); // Wait for a keystroke in the window
}
