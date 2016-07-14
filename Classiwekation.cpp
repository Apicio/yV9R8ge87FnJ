#include "Classiwekation.h"


Classiwekation::Classiwekation(void)
{
    JavaVMInitArgs vm_args;
	JNI_GetDefaultJavaVMInitArgs(&vm_args);
	JavaVMOption* options = new JavaVMOption[1] ;
	options[0].optionString = OPTION_STRING; //Path to the java source code
    vm_args.version = JNI_VERSION_1_8; //JDK version. This indicates version 1.6
    vm_args.nOptions = 1;
    vm_args.options = options;
    vm_args.ignoreUnrecognized = false;

    int ret = JNI_CreateJavaVM(&jvm, (void**)&env, &vm_args);
    if(ret < 0)
    	printf("\nUnable to Launch JVM\n");

	clsWeka = env->FindClass("Weka"); // Read Weka.class
	runClassification = env->GetMethodID(clsWeka,"runClassification","(Ljava/lang/String;)D");
	jmethodID clsWekaConst = env->GetMethodID(clsWeka, "<init>", "(Ljava/lang/String;)V");
	jstring StringArg = env->NewStringUTF(MODEL);
	WekaObj = env->NewObject(clsWeka, clsWekaConst, StringArg);
}

double Classiwekation::classify(string features){	
	jstring StringArg = env->NewStringUTF(features.c_str());
	double jobjRetData = env->CallDoubleMethod(WekaObj,runClassification,StringArg);
	jthrowable exc = env->ExceptionOccurred();
    if (exc) {
        jclass newExcCls;
        env->ExceptionDescribe();
        env->ExceptionClear();
    }
	return jobjRetData;
}

void Classiwekation::ClassTest(void){
	struct ControlDetail ctrlDetail;	
	ctrlDetail.ID = 11;
	strcpy(ctrlDetail.Name,"HR-HW");
	strcpy(ctrlDetail.IP,"10.32.164.133");
	ctrlDetail.Port = 9099;
	
	printf("Struct Created in C has values:\nID:%d\nName:%s\n IP:%s\nPort:%d\n",ctrlDetail.ID,ctrlDetail.Name,ctrlDetail.IP,ctrlDetail.Port);

	/********************************************************/
	struct WorkOrder WO[2];
	strcpy(WO[0].sumSerialId,"2000");
	strcpy(WO[0].accessNumber,"2878430");
	strcpy(WO[0].actionType,"04");
	strcpy(WO[0].effectiveDate,"25-12-2007 12:20:30 PM");
	strcpy(WO[0].fetchFlag, "0");
	strcpy(WO[0].reason,"Executed Successfully");
	strcpy(WO[0].accessSource,"PMS");
	strcpy(WO[1].sumSerialId,"1000");
	strcpy(WO[1].accessNumber,"2878000");
	strcpy(WO[1].actionType,"T4");
	strcpy(WO[1].effectiveDate,"25-12-2007 11:20:30 PM");
	strcpy(WO[1].fetchFlag,"0");
	strcpy(WO[1].reason,"");
	strcpy(WO[1].accessSource,"RMS");

	
    jclass clsH=NULL;
    jclass clsC = NULL;
    jclass clsW = NULL;
	jclass clsR = NULL;
	jclass weka = NULL;
    jmethodID midMain = NULL;
    jmethodID midCalling = NULL;
    jmethodID midDispStruct = NULL;
    jmethodID midDispStructArr = NULL;
	jmethodID midRetObjFunc = NULL;
    jmethodID midCtrlDetConst = NULL;
    jmethodID midWoConst = NULL;
    
    jobject jobjDet = NULL;
	jobject jobjRetData = NULL;
    jobjectArray jobjWOArr = NULL;
    
    //Obtaining Classes
    clsH = env->FindClass("HelloWorld");
    clsC = env->FindClass("ControlDetail");
    clsW = env->FindClass("WorkOrder");

	weka = env->FindClass("HelloWeka");
	midMain = env->GetStaticMethodID(weka, "main", "([Ljava/lang/String;)V");
	env->CallStaticVoidMethod(weka, midMain, NULL); //Calling the main method.

	jthrowable exc;
	exc = env->ExceptionOccurred();
     if (exc) {
         jclass newExcCls;
         env->ExceptionDescribe();
         env->ExceptionClear();

     }
	
	//Obtaining Method IDs
    if (clsH != NULL)
    {
		midMain       = env->GetStaticMethodID(clsH, "main", "([Ljava/lang/String;)V");
		midCalling    = env->GetStaticMethodID(clsH,"TestCall","(Ljava/lang/String;)V");
		midDispStruct = env->GetStaticMethodID(clsH,"DisplayStruct","(LControlDetail;)I");
		midDispStructArr = env->GetStaticMethodID(clsH,"DisplayStructArray","([LWorkOrder;)V");
		midRetObjFunc = env->GetStaticMethodID(clsH,"ReturnObjFunc","()Ljava/lang/Object;");
	}
	else
    {
    	printf("\nUnable to find the requested class\n");    	
    }
	if(clsC != NULL)
	{
		//Get constructor ID for ControlDetail
		midCtrlDetConst = env->GetMethodID(clsC, "<init>", "(ILjava/lang/String;Ljava/lang/String;I)V");		
	}
	else
    {
    	printf("\nUnable to find the requested class\n");    	
    }	

	if(clsW != NULL)
	{
		//Get Constructor ID for WorkOrder
		midWoConst = env->GetMethodID(clsW, "<init>", "(Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;Ljava/lang/String;)V");		
	}
	else
    {
    	printf("\nUnable to find the requested class\n");    	
    }

	/************************************************************************/
	/* Now we will call the functions using the their method IDs			*/
	/************************************************************************/
	if(midMain != NULL)
		env->CallStaticVoidMethod(clsH, midMain, NULL); //Calling the main method.
	
	if (midCalling!=NULL)
	{
		jstring StringArg = env->NewStringUTF("\nTestCall:Called from the C Program\n");
		//Calling another static method and passing string type parameter
		env->CallStaticVoidMethod(clsH,midCalling,StringArg);
	}
	
	printf("\nGoing to Call DisplayStruct\n");
	if (midDispStruct!=NULL)
	{
		if(clsC != NULL && midCtrlDetConst != NULL)
		{
			jstring StringArgName = env->NewStringUTF(ctrlDetail.Name);
			jstring StringArgIP = env->NewStringUTF(ctrlDetail.IP);
			
			//Creating the Object of ControlDetail.
			jobjDet = env->NewObject(clsC, midCtrlDetConst, (jint)ctrlDetail.ID, StringArgName, StringArgIP, (jint)ctrlDetail.Port);
		}
		
		if(jobjDet != NULL && midDispStruct != NULL)
			env->CallStaticIntMethod(clsH,midDispStruct,jobjDet); //Calling the method and passing ControlDetail Object as parameter
	}
	//Calling a function from java and passing Structure array to it.
	printf("\n\nGoing to call DisplayStructArray From C\n\n");
	if (midDispStructArr!=NULL)
	{
		if(clsW != NULL && midWoConst != NULL)
		{
			//Creating the Object Array that will contain 2 structures.
			jobjWOArr = (jobjectArray)env->NewObjectArray(2,clsW,env->NewObject(clsW, midWoConst,env->NewStringUTF(""),env->NewStringUTF(""),env->NewStringUTF(""),
														  env->NewStringUTF(""),env->NewStringUTF(""),env->NewStringUTF(""),env->NewStringUTF("")));         	
			//Initializing the Array
			for(int i=0;i<2;i++)
			{
				env->SetObjectArrayElement(jobjWOArr,i,env->NewObject(clsW, midWoConst,env->NewStringUTF(WO[i].sumSerialId),
														env->NewStringUTF(WO[i].accessNumber),
														env->NewStringUTF(WO[i].actionType),
														env->NewStringUTF(WO[i].effectiveDate),
														env->NewStringUTF(WO[i].fetchFlag),
														env->NewStringUTF(WO[i].reason),
														env->NewStringUTF(WO[i].accessSource)));	
			}
		}
		//Calling the Static method and passing the Structure array to it.
		if(jobjWOArr != NULL && midDispStructArr != NULL)
			env->CallStaticVoidMethod(clsW,midDispStructArr,jobjWOArr);
	}
	//Calling a Static function that return an Object
	if (midRetObjFunc != NULL)
	{
		//Calling the function and storing the return object into jobject type variable
		//Returned object is basically a structure having two fields (string and integer)
		jobjRetData = (jobject)env->CallStaticObjectMethod(clsH,midRetObjFunc,NULL);
		//Get the class of object
		clsR = env->GetObjectClass(jobjRetData);
		//Obtaining the Fields data from the returned object
		jint nRet = env->GetIntField(jobjRetData,env->GetFieldID(clsR,"returnValue","I"));
		jstring jstrLog = (jstring)env->GetObjectField(jobjRetData,env->GetFieldID(clsR,"Log","Ljava/lang/String;"));
		const char *pLog = env->GetStringUTFChars(jstrLog,0);
		
		printf("\n\nValues Returned from Object are:\nreturnValue=%d\nLog=%s",nRet,pLog);
		//After using the String type data release it.
		env->ReleaseStringUTFChars(jstrLog,pLog);
	}
}


Classiwekation::~Classiwekation(void)
{
	int n = jvm->DestroyJavaVM();
}
