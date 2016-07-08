
// Aldebaran includes.
#include <alproxies/almotionproxy.h>
#include <alproxies/alrobotpostureproxy.h>
#include <alproxies/alvideodeviceproxy.h>
#include <alvision/alimage.h>
#include <alvision/alvisiondefinitions.h>
#include <alerror/alerror.h>

// Opencv includes.
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

// Other includes.
#include <math.h>
#include <time.h>
#include <stdlib.h>
#include <iostream>
#include <sstream>


#define NAOIP "169.254.200.36"
#define NAOPORT 9559
#define SAVEPATH "C:\\imgCatt\\"

using namespace AL;
class NaoUtils
{

public:
	NaoUtils(void);
	~NaoUtils(void);
	void explore();
	void writeImages(const std::string& naoIP, const std::string& path);
};

