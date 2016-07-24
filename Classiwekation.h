#pragma once
#include <stdio.h>
#include <vector>
#include <jni.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include "detection.h"
#include "FeatExtract.h"
#define MODEL "..\\JavaSrc\\TestStruct\\"
using namespace std;

#define PATH_SEPARATOR ';' /* define it to be ':' on Solaris */
#define USER_CLASSPATH "." /* where Prog.class is */
#define OPTION_STRING "-Djava.class.path=C:\\Users\\leo\\Desktop\\NaoVision\\JavaSrc\\TestStruct;C:\\Users\\leo\\Desktop\\NaoVision\\JavaSrc\\TestStruct\\weka.jar"

class Classiwekation
{
private:
	jclass clsWeka;
	jmethodID runClassification;
	jobject WekaObj;
	JNIEnv *env;
	JavaVM * jvm;
public:
	Classiwekation(void);
	void ClassTest(void);
	double classify(string featuresBGR, string featuresLBP );
	double recognition(std::vector<cv::Mat>); // Ogni 3 abbiamo una direzione.
	~Classiwekation(void);
};

