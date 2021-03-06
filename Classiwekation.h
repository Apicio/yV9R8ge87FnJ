#pragma once
#include <stdio.h>
#include <jni.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#define MODEL "..\\JavaSrc\\TestStruct\\"
using namespace std;

#define PATH_SEPARATOR ';' /* define it to be ':' on Solaris */
#define USER_CLASSPATH "." /* where Prog.class is */
#define OPTION_STRING "-Djava.class.path=C:\\Users\\leo\\Desktop\\LAST_NaoVision\\JavaSrc\\TestStruct;C:\\Users\\leo\\Desktop\\LAST_NaoVision\\JavaSrc\\TestStruct\\weka.jar;C:\\Users\\leo\\Desktop\\LAST_NaoVision\\JavaSrc\\TestStruct\\LibSVM.jar;C:\\Users\\leo\\Desktop\\LAST_NaoVision\\JavaSrc\\TestStruct\\libsvm2.jar"

struct ControlDetail
{
	int ID;
	char Name[100];
	char IP[100];
	int Port;
};

struct WorkOrder
{
	char		sumSerialId[20];	
	char		accessNumber[18];
	char		actionType[4];
	char		effectiveDate[24];
	char		fetchFlag[2];
	char		reason[456];
	char		accessSource[100];	
};

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
	double classify(string features);
	~Classiwekation(void);
};

