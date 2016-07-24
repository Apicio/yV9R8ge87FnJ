#pragma once

#include <iostream>
#include <sstream>

// Aldebaran includes.
#include <alerror/alerror.h>
#include <alproxies/almemoryproxy.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
// Opencv includes.
#include <opencv2/core/core.hpp>
#include <math.h>
#include <opencv2/highgui/highgui.hpp>
#include <time.h>
#include <stdlib.h>

#define NAOIP "192.168.0.116"
#define NAOPORT 9559
#define SAVEPATH "C:\\imgCatt\\"
using namespace AL;
using namespace cv;
class NaoUtils
{

public:
	int currCameraID;
	string subscriberID;
	ALValue _imger;

	NaoUtils(void);
	~NaoUtils(void);
	void cameraSwitch(ALVideoDeviceProxy camProx);
	void explore();
	void init();
	void writeImages(const std::string& naoIP, const std::string& path);
	void takeSomePhotos(std::string path);
	Mat see(ALVideoDeviceProxy, ALValue&);
	Mat seeDetection(ALVideoDeviceProxy camProx, ALValue&);
	void camShutdown(ALVideoDeviceProxy camProx);
};

