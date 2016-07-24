#include "NaoUtils.h"


NaoUtils::NaoUtils(void)
{}

void NaoUtils::explore(){

	/** The name of the joint to be moved. */
	const AL::ALValue jointYaw = "HeadYaw";
	const AL::ALValue jointPitch = "HeadPitch";

	try {
		AL::ALMotionProxy motionProx(NAOIP, NAOPORT);
		AL::ALRobotPostureProxy posture(NAOIP, NAOPORT);
		posture.goToPosture("Stand",1.0); 


		/** Create a ALMotionProxy to call the methods to move NAO's head.
		* Arguments for the constructor are:
		* - IP adress of the robot
		* - port on which NAOqi is listening, by default 9559
		*/


		/** Make sure the head is stiff to be able to move it.
		* To do so, make the stiffness go to the maximum in one second.
		*/
		/** Target stiffness. */
		AL::ALValue stiffness = 1.0f;
		/** Time (in seconds) to reach the target. */
		AL::ALValue time = 1.0f;
		/** Call the stiffness interpolation method. */
		motionProx.stiffnessInterpolation(jointYaw, stiffness, time);
		motionProx.stiffnessInterpolation(jointPitch, stiffness, time);

		/** Set the target angle list, in radians. */
		AL::ALValue targetAnglesPitch = AL::ALValue::array(0.4f);
		/** Set the corresponding time lists, in seconds. */
		AL::ALValue targetTimesPitch = AL::ALValue::array(1.0f);
		bool isAbsolute = true;
		motionProx.post.angleInterpolation(jointPitch, targetAnglesPitch, targetTimesPitch, isAbsolute);

		AL::ALValue targetAnglesYaw = AL::ALValue::array(-1.2f,1.2f,0.0f);
		AL::ALValue targetTimesYaw = AL::ALValue::array(7.0f,11.0f,15.0f);

		/** Call the angle interpolation method. The joint will reach the
		* desired angles at the desired times.
		*/
		motionProx.post.angleInterpolation(jointYaw, targetAnglesYaw, targetTimesYaw, isAbsolute);
		writeImages(NAOIP,SAVEPATH);
		/** Remove the stiffness on the head. */
		stiffness = 0.0f; time = 3.5f;
		motionProx.stiffnessInterpolation(jointPitch, stiffness, time);
		motionProx.stiffnessInterpolation(jointYaw, stiffness, time);
		motionProx.rest();

	}
	catch (const AL::ALError& e) {
		std::cerr << "Caught exception: " << e.what() << std::endl;
		exit(1);
	}
	exit(0);
}

Mat NaoUtils::seeDetection(ALVideoDeviceProxy camProx, ALValue &img){
	cv::Mat imgHeader;
	try{
	std::string subscriberID = "subscriberID2";
	/** Subscribe a client image requiring 640*480 and BGR colorspace.*/
	subscriberID = camProx.subscribeCamera("subscriberID2",1, kVGA, kBGRColorSpace, 30);
	/** Create an cv::Mat header to wrap into an opencv image.*/
	imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC3);


	/** Main loop. Exit when pressing ESC.*/
	img = camProx.getImageRemote(subscriberID);
	imgHeader.data = (uchar*) img[6].GetBinary();
	camProx.releaseImage(subscriberID);
	/** Cleanup.*/
	camProx.unsubscribe(subscriberID);
	}catch(std::exception& e){std::cout<<e.what()<<std::endl; camProx.unsubscribe(subscriberID);
	}
	return imgHeader.clone();
}


Mat NaoUtils::see(ALVideoDeviceProxy camProx, ALValue &img){
	cv::Mat imgHeader;
	try{
	std::string subscriberID = "subscriberID1";
	/** Subscribe a client image requiring 640*480 and BGR colorspace.*/
	subscriberID = camProx.subscribeCamera("subscriberID1",1, kQVGA, kBGRColorSpace, 30);
	/** Create an cv::Mat header to wrap into an opencv image.*/
	imgHeader = cv::Mat(cv::Size(320, 240), CV_8UC3);


	/** Main loop. Exit when pressing ESC.*/
	img = camProx.getImageRemote(subscriberID);
	imgHeader.data = (uchar*) img[6].GetBinary();
	camProx.releaseImage(subscriberID);

	static int i = 0;
	i++;
	std::stringstream ss;
	ss << "prova\\image_14" << i <<".jpg";
	imwrite(ss.str(),imgHeader);
	imshow("naosee",imgHeader);

	/** Cleanup.*/
	camProx.unsubscribe(subscriberID);
	}catch(std::exception& e){std::cout<<e.what()<<std::endl; camProx.unsubscribe(subscriberID);
	}
	return imgHeader;
}

void NaoUtils::takeSomePhotos(std::string path){
    AL::ALMotionProxy motionProx(NAOIP, NAOPORT);
	AL::ALRobotPostureProxy posture(NAOIP, NAOPORT);
	posture.goToPosture("Stand",1.0);
	ALVideoDeviceProxy camProx(NAOIP, NAOPORT);
	std::string subscriberID = "subscriberID";
	/** Subscribe a client image requiring 640*480 and BGR colorspace.*/
	subscriberID = camProx.subscribeCamera("subscriberID",1, kVGA, kBGRColorSpace, 30);
	/** Create an cv::Mat header to wrap into an opencv image.*/
	cv::Mat imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC3);
	std::stringstream ss;
	int keyPressed=0;
	/** Main loop. Exit when pressing ESC.*/
	while(keyPressed != 'e'){
		srand(time(NULL));
		ALValue img = camProx.getImageRemote(subscriberID);
		imgHeader.data = (uchar*) img[6].GetBinary();
		ss<<path<<"img"<< rand() <<".jpg";
		cv::imshow("curr", imgHeader);
		keyPressed=cv::waitKey(300);
		cv::imwrite(ss.str(),imgHeader);
		ss.str("");
		camProx.releaseImage(subscriberID);
	}
	/** Cleanup.*/
	camProx.unsubscribe(subscriberID);
	motionProx.rest();

}



void NaoUtils::writeImages(const std::string& naoIP, const std::string& path){
	ALVideoDeviceProxy camProx(NAOIP, NAOPORT);
	std::string subscriberID = "subscriberID";
	/** Subscribe a client image requiring 640*480 and BGR colorspace.*/
	subscriberID = camProx.subscribeCamera("subscriberID",0, kVGA, kBGRColorSpace, 30);

	/** Create an cv::Mat header to wrap into an opencv image.*/
	cv::Mat imgHeader = cv::Mat(cv::Size(640, 480), CV_8UC3);
	srand(time(NULL));
	/** Main loop. Exit when pressing ESC.*/
	for(int i=0;i<15;i++){
		std::stringstream ss;
		ALValue img = camProx.getImageRemote(subscriberID);

		/** Access the image buffer (6th field) and assign it to the opencv image
		* container. */
		imgHeader.data = (uchar*) img[6].GetBinary();
		ss<<path<<"img"<< rand() <<".jpg";
		cv::imwrite(ss.str(),imgHeader);

		/** Tells to ALVideoDevice that it can give back the image buffer to the
		* driver. Optional after a getImageRemote but MANDATORY after a getImageLocal.*/
		camProx.releaseImage(subscriberID);

	}
	/** Cleanup.*/
	camProx.unsubscribe(subscriberID);
}



NaoUtils::~NaoUtils(void)
{
}
