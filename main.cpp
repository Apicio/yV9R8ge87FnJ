/**
 * Copyright (c) 2011 Aldebaran Robotics. All Rights Reserved
 * \file sayhelloworld.cpp
 * \brief Make NAO say a short phrase.
 *
 * A simple example showing how to make NAO say a short phrase using the
 * specialized proxy ALTextToSpeechProxy.
 */

#include <iostream>
#include <alerror/alerror.h>
#include <alproxies/altexttospeechproxy.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <iostream>

using namespace cv;
using namespace std;

int main(int argc, char* argv[])
{
	const string img_file = "dice.jpg";
	Mat image;
	image = imread(img_file,  IMREAD_COLOR); // Read the file

	if (!image.data) // Check for invalid input
	{
		cout << "Could not open or find the image" << std::endl;
		return -1;
	}

	namedWindow("Display window", WINDOW_AUTOSIZE); // Create a window for display.
	imshow("Display window", image); // Show our image inside it.

	waitKey(0); // Wait for a keystroke in the window
}
