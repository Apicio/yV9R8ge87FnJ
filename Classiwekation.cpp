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
	runClassification = env->GetMethodID(clsWeka,"runClassification","(Ljava/lang/String;Ljava/lang/String;)D");
	jmethodID clsWekaConst = env->GetMethodID(clsWeka, "<init>", "(Ljava/lang/String;)V");
	jstring StringArg = env->NewStringUTF(MODEL);
	WekaObj = env->NewObject(clsWeka, clsWekaConst, StringArg);

}

double Classiwekation::classify(string featuresBGR, string featuresLBP){	
	jstring StringArgBGR = env->NewStringUTF(featuresBGR.c_str());
	jstring StringArgLBP = env->NewStringUTF(featuresLBP.c_str());
	double jobjRetData = env->CallDoubleMethod(WekaObj,runClassification,StringArgBGR,StringArgLBP);
	jthrowable exc = env->ExceptionOccurred();
    if (exc) {
        jclass newExcCls;
        env->ExceptionDescribe();
        env->ExceptionClear();
    }
	return jobjRetData;
}

double Classiwekation::recognition(std::vector<cv::Mat> buffer, TheWalkingNao& twn){
	// cambiare risoluzione
	cout<<"Calling recognition"<<endl;
	vector<vector<double> > results;
	Mat kernelOp = getStructuringElement(MORPH_RECT,Size(7,7));
	Direction stash1; Point stash2; float stash3=0;
	vector<Mat> rectangles;
	for(int i=0; i<buffer.size(); i++){
		cv::Mat image = buffer[i];		
		vector<Blob> blobs;		
		cout<<"filtering image "<< endl;
		Mat path_mask;
		twn.pathfinder(image, stash1, stash2, stash3, path_mask);	
		dilate(path_mask,path_mask,kernelOp);	
		cout<<"calling detection"<<endl;
		detect2(image,rectangles, blobs, path_mask);
		cout<<"Detection done with "<< blobs.size() << "candidates elements" << endl;
		for(int k = 0; k<blobs.size(); k++){
			cout<<"Extracting features"<<endl;
			// scarta blob a y < 140 (cambiare risoluzione)
			// scartare blob piccoli
			string featuresBGR = FeatExtract().extractForColorClassifier(blobs[i], false);
			string featuresLBP = FeatExtract().extractForLBPClassifier(blobs[i], false);
			results.at(i).push_back(classify(featuresBGR, featuresLBP));
			// aggiungere una probabilit� in funzione della dimensione del blob, gli oggetti di interesse hanno tutti una certa dimensione
			// potrebbero esserci ogetti sullo sfondo per marker di stop diversi da quello corrente, filtra distanza.
		}
	}
	int red_apple = 0;
	int yellow_apple = 0;
	int glass = 0;
	int coffe_cup = 0;
	for(int i=0; i<results.size(); i++){
		cout << "Analise Shots.. ";
		vector<double> frame_result = results.at(i);
			for(int k=0; k<frame_result.size(); k++){
				cout << "Case" << i <<", " << k << endl;
				switch((int)frame_result.at(k)){
				case 0:
					red_apple++;
					break;
				case 1:
					yellow_apple++;
					break;
				case 2:
					glass++;
					break;
				case 3:
					coffe_cup++;
					break;
				}
			}
	}
	// in input abbiamo 9 foto, 3 per direzione.
	// dtection per ognuna
	// feature extraction
	// clasidicazione su gruppi di 3
	// si decide a maggioranza su una direzione
	return rand()%4;
}

Classiwekation::~Classiwekation(void)
{
	int n = jvm->DestroyJavaVM();
}
