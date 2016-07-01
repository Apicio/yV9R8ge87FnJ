#pragma once

#include <iostream>
#include <sstream>

// Aldebaran includes.
#include <alerror/alerror.h>
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>
// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#define NAOIP "192.168.1.101"
#define NAOPORT 9559
#define SAVEPATH "C:\\imgCatt\\"
using namespace AL;
using namespace std;
class NaoUtils
{

public:
	NaoUtils(void);
	~NaoUtils(void);
	void explore();
	void writeImages(const std::string& naoIP, const std::string& path);

};

