#pragma once
#include <stdio.h>
#include <jni.h>
#include <string.h>

#define PATH_SEPARATOR ';' /* define it to be ':' on Solaris */
#define USER_CLASSPATH "." /* where Prog.class is */
#define OPTION_STRING "-Djava.class.path=\\JavaSrc\\TestStruct;\\JavaSrc\\TestStruct\\weka.jar"

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
public:
	JNIEnv *env;
	JavaVM * jvm;

	Classiwekation(void);
	void ClassTest(void);
	~Classiwekation(void);
};

