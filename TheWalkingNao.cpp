#include "TheWalkingNao.h"
#define NAO 1

TheWalkingNao::TheWalkingNao(){
/* Configuration */
	_ImageSharp = true;
	_SharpSigma = 20;
	_SharpThreshold = 5;
	_SharpAmount = 3;
	_medianBlur = 11;
	_markSize = 0.105;
	_invert = false;
/* Init */
/*	Mat distorsionCoeff=cv::Mat::zeros(5,1,CV_32FC1);
	Mat cameraMatrix=cv::Mat::eye(3,3,CV_32FC1);
		cameraMatrix.at<float>(0,0)=1406.9968064577436;
	cameraMatrix.at<float>(0,1)=0;
	cameraMatrix.at<float>(0,2)=639.5;
	cameraMatrix.at<float>(1,0)=0;
	cameraMatrix.at<float>(1,1)=1406.9968064577436;
	cameraMatrix.at<float>(1,2)=479.5;
	cameraMatrix.at<float>(2,0)=0;
	cameraMatrix.at<float>(2,1)=0;
	cameraMatrix.at<float>(2,2)=1;
	distorsionCoeff.at<float>(0,0)=-0.48289968517339044;;
	distorsionCoeff.at<float>(1,0)=0.88748438503520599;
	distorsionCoeff.at<float>(2,0)=0;
	distorsionCoeff.at<float>(3,0)=0;
	distorsionCoeff.at<float>(4,0)=0.33637972422994983;
	/*
	cameraMatrix.at<float>(0,0)=558.570339530768;
	cameraMatrix.at<float>(0,1)=0;
	cameraMatrix.at<float>(0,2)=308.885375457296;
	cameraMatrix.at<float>(1,0)=0;
	cameraMatrix.at<float>(1,1)=556.122943034837;
	cameraMatrix.at<float>(1,2)=247.600724811385;
	cameraMatrix.at<float>(2,0)=0;
	cameraMatrix.at<float>(2,1)=0;
	cameraMatrix.at<float>(2,2)=1;
	distorsionCoeff.at<float>(0,0)=-0.0648763971625288;
	distorsionCoeff.at<float>(1,0)=0.0612520196884308;
	distorsionCoeff.at<float>(2,0)=0.0038281538281731;
	distorsionCoeff.at<float>(3,0)=-0.00551104078371959;
	distorsionCoeff.at<float>(4,0)=0;*/
	Mat distorsionCoeff=cv::Mat::zeros(5,1,CV_32FC1);
	Mat cameraMatrix=cv::Mat::eye(3,3,CV_32FC1);
	cameraMatrix.at<float>(0,0)=1.2013236249865872e+003;
	cameraMatrix.at<float>(0,1)=0;
	cameraMatrix.at<float>(0,2)=3.1950000000000000e+002;
	cameraMatrix.at<float>(1,0)=0;
	cameraMatrix.at<float>(1,1)=1.2013236249865872e+003;
	cameraMatrix.at<float>(1,2)=2.3950000000000000e+002;
	cameraMatrix.at<float>(2,0)=0;
	cameraMatrix.at<float>(2,1)=0;
	cameraMatrix.at<float>(2,2)=1;
	distorsionCoeff.at<float>(0,0)=1.8523503367065681e+000;
	distorsionCoeff.at<float>(1,0)=-2.5020691022262803e+001;
	distorsionCoeff.at<float>(2,0)=0;
	distorsionCoeff.at<float>(3,0)=0;
	distorsionCoeff.at<float>(4,0)=3.0809776264102879e+002;
	Size resolution(WIDTH,HEIGHT);
	CameraParameters cam(cameraMatrix, distorsionCoeff, resolution);
	camParams = cam;
}

vector<Marker> TheWalkingNao::ArucoFind(Mat img, double& angle, bool toRemoveMarkers){
	if(_invert){
		Mat white(img.size(), img.type(), Scalar(255,255,255));
		img = white - img;
	}
	  vector<Marker> Markers;
    try{
/* Declaration */
        MarkerDetector MDetector;
      
        Mat grayImage, croppedImage;
		Mat thresholded = Mat::zeros(img.size(),img.type());
		Mat toReturn = img.clone();
		vector< vector<Point> > contours;
		static int k = 0;
/* Marker Detect */
		if(_ImageSharp){
			Mat blurred; 
			GaussianBlur(img, blurred, Size(), _SharpSigma, _SharpSigma);
			Mat lowContrastMask = abs(img - blurred) < _SharpThreshold;
			Mat sharpened = img*(1+_SharpAmount) + blurred*(-_SharpAmount);
			img.copyTo(sharpened, lowContrastMask);
			sharpened.copyTo(img);
			/*
			GpuMat blurred, src;
			src.upload(img);
			gpu::GaussianBlur(src, blurred, Size(), _SharpSigma, _SharpSigma);
			GpuMat lowContrastMask = gpu::abs(src - blurred) < _SharpThreshold;
			GpuMat sharpened = src*(1+_SharpAmount) + blurred*(-_SharpAmount);
			src.copyTo(sharpened, lowContrastMask);
			sharpened.copyTo(src);
			src.download(img);*/
		}
/*			if(_ImageSharp){
			GpuMat blurred, src, dest, sharpened;
			Mat lowCM, ssharp, srcMat;

			src.upload(img);
			gpu::GaussianBlur(src, blurred, Size(), _SharpSigma, _SharpSigma);
			GpuMat lowContrastMask;
			gpu::absdiff(src,blurred,lowContrastMask);
			gpu::multiply(src, (1+_SharpAmount), src);
			gpu::multiply(blurred, (-_SharpAmount), blurred);
			gpu::add(src, blurred, sharpened); 		
			lowContrastMask.download(lowCM);
			sharpened.download(ssharp);
			src.download(srcMat);

			lowCM = lowCM < _SharpThreshold;
			srcMat.copyTo(ssharp, lowCM);
			ssharp.copyTo(img);
		}*/
#if NAO
			MDetector.detect(img,Markers,camParams,_markSize,false);	
#else
			MDetector.detect(img,Markers);	
#endif
/* for each marker, draw info and its boundaries in the image */
		for (unsigned int i=0;i<Markers.size();i++) {
			double v1[] = {Markers[i].at(0).x, Markers[i].at(1).x, Markers[i].at(2).x, Markers[i].at(3).x};
			double v2[] = {Markers[i].at(0).y, Markers[i].at(1).y, Markers[i].at(2).y, Markers[i].at(3).y};
			double maxX = fmax(v1,4);
			double maxY = fmax(v2,4);
			double minX = fmin(v1,4);
			double minY = fmin(v2,4);
			for(int x = minY; x<maxY; x++){
				for(int y = minX; y<maxX; y++){
					if(pnpoly(4,v1,v2,y,x))
						thresholded.at<char>(x,y) = 255;
				}
			}
#if NAO
			cvtColor(img,thresholded,CV_BGR2GRAY);
			threshold(thresholded, thresholded,220,255,THRESH_BINARY);
			Size s = thresholded.size();
			medianBlur(thresholded,thresholded,_medianBlur);
			findContours(thresholded, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
			int minContour;
			double distance = INT_MAX;
			for (int idx = 0; idx < contours.size(); idx++){
				vector<Point> border = contours.at(idx);
				Point p(0,0); 
				for(int jdx = 0; jdx < border.size(); jdx++){
					p.x +=border.at(jdx).x;
					p.y +=border.at(jdx).y;
				}	
				p.x /= border.size();
				p.y /= border.size();
				double currDist = cv::norm((Point)Markers[i].getCenter()-p);;
				if(currDist < distance){
					distance = currDist;
					minContour = idx;
				}	
			}
			drawContours(toReturn,contours,minContour,Scalar(0,0,0),CV_FILLED);
			
			Markers[i].draw(img,Scalar(0,0,255),2);
			CvDrawingUtils u;
			u.draw3dAxis(img,Markers[i],camParams);		
			cout<<"MARK  "<<Markers[i]<<endl;
				//TODO:
				/*Aruco rileva UN solo Marker per ID, è possibile che nella stessa scena vi siano
				due marker, quindi l'idea è quella di utilizzare la posizione del centroide. Questa
				cosa è da fare se e solo se abbiamo il problema del rilevamento fra più marker*/
				//if(Markers[i].id == 136)
			angle = computeAngle(Markers[i],camParams);
			cout << Markers[i].id <<" "<< camParams.getCameraLocation(Markers[i].Rvec,Markers[i].Tvec) << endl;
			u.draw3dAxis(img,Markers[i],camParams);
			imshow("m",img);
#else
			Markers[i].draw(img,Scalar(0,0,255),2);
#endif					
		}
#if NAO
		vector<std::vector<cv::Point2f> > candidates = MDetector.getCandidates();
		for(unsigned int kk = candidates.size()-1; kk>=0 && Markers.size() !=0; kk--){
			std::vector<cv::Point2f> candidate =  candidates.at(kk);
			Point cent(0,0);
			for(unsigned int ii = 0; ii<candidate.size(); ii++){
				cent.x += candidate.at(ii).x;
				cent.y += candidate.at(ii).y;
			}

			cent.x /=4;
			cent.y /=4;
			for(unsigned int i=0;i<Markers.size();i++){
				if(cv::norm((Point)Markers[i].getCenter()-cent) < 10){
					candidates.erase (candidates.begin()+kk);
					break;
				}
			}
		} 
		for(int i=0; i<candidates.size(); i++){
			MDetector.drawLine(img,candidates,i);
		}

		vector<std::vector<cv::Point2f> > cand = MDetector.getCandidates();
		for(int i=0; i<cand.size(); i++){
			MDetector.drawLine(img,cand,i);
		}
#endif	
		
    }catch (std::exception &ex){cout<<"Exception :"<<ex.what()<<endl;}
	return Markers;
}
int TheWalkingNao::pnpoly(int nvert, double *vertx, double *verty, double testx, double testy)
{
  int i, j, c = 0;
  for (i = 0, j = nvert-1; i < nvert; j = i++) {
    if ( ((verty[i]>testy) != (verty[j]>testy)) &&
     (testx < (vertx[j]-vertx[i]) * (testy-verty[i]) / (verty[j]-verty[i]) + vertx[i]) )
       c = !c;
  }
  return c;
}


double TheWalkingNao::computeAngle(Marker m, CameraParameters cam){
	vector<Point2f> imagePoints;
	float size=m.ssize*3;
	Mat objectPoints (4,3,CV_32FC1);
	objectPoints.at<float>(0,0)=0;
	objectPoints.at<float>(0,1)=0;
	objectPoints.at<float>(0,2)=0;
	objectPoints.at<float>(1,0)=size;
	objectPoints.at<float>(1,1)=0;
	objectPoints.at<float>(1,2)=0;
	objectPoints.at<float>(2,0)=0;
	objectPoints.at<float>(2,1)=size;
	objectPoints.at<float>(2,2)=0;
	objectPoints.at<float>(3,0)=0;
	objectPoints.at<float>(3,1)=0;
	objectPoints.at<float>(3,2)=size;
	cv::projectPoints( objectPoints, m.Rvec, m.Tvec, cam.CameraMatrix,cam.Distorsion,imagePoints);
	Point2f p = imagePoints[1]-imagePoints[0];
	return 180*(atan2(-p.y,p.x))/M_PI;
}

double TheWalkingNao::fmin(double element[], int size){
	double _small = INT_MAX;
	for(int i=0; i<size; i++){
		if (element[i] < _small)
			_small = element[i];
	}
	return _small;
}
double TheWalkingNao::fmax(double element[], int size){
	double _small = element[0];
	for(int i=0; i<size; i++){
		if (element[i] > _small)
			_small = element[i];
	}
	return _small;
}
void TheWalkingNao::init(const char* robotIP){
	this->motion = new AL::ALMotionProxy(robotIP,PORT); 
	this->robotPosture = new AL::ALRobotPostureProxy(robotIP,PORT);
}
void TheWalkingNao::standUp() {
	/* required position before moving */
 	robotPosture->goToPosture("StandInit", 0.5);
 }
 
 void TheWalkingNao::moveLeft(float meters,double angle) {
 	/* moves to the left, rotating torso 90 deg. counter-clockwise */
	motion->moveTo(0,0,angle);
 	motion->moveTo(meters, 0, 0);
 }
 
 void TheWalkingNao::moveRight(float meters,double angle) {
 	/* moves to the right, rotating torso 90 deg. clockwise */
	motion->moveTo(0, 0, -angle);
 	motion->moveTo(meters, 0, 0);
 }
 
 void TheWalkingNao::moveForward(float meters) {
 	/* moves forward, without torso rotation */
	 motion->moveTo(meters, 0, 0);//{ {"MaxStepX",0.02} , {"MaxStepY",0.101} });
 }

 void TheWalkingNao::moveOnX(float meters){
	 motion->moveTo(0, meters, 0);
 }
 void TheWalkingNao::rotate(float angle){
	 motion->moveTo(0, 0, angle);
 }
 void TheWalkingNao::walk(float X, float Y){
	 motion->moveTo(X,Y,0);
 }

  void TheWalkingNao::restNow() {
 	/* moves forward, without torso rotation */
 	motion->rest();
 }
bool TheWalkingNao::isMoving(){
	return motion->moveIsActive();
}



 void TheWalkingNao::moveNearMarker(Mat img){
	double angle = 0;
	double distY = 0; 
	double distX = 0;
	int d = 0;
	int heigh = 0;
	vector<Marker> markers ;
	/*Becca il marker*/
	/*Vai avanti finché non scompare salvando l'angolo*/
	/*ruota dell'angolo che hai ottenuto*/
	do{		
		markers = ArucoFind(img,angle,false);
		
		if( markers.size() != 0){
			cout<<markers[0].getCenter()<<endl;
			heigh = markers[0].getPerimeter()/4; //supponiamo mm
		}
		distX = markers.size() != 0 ? ( markers[0].getCenter().x - WIDTH/2 ) : distX*0.8; 
	/*	distY = markers.size() != 0 ?( HEIGH - markers[0].getCenter().y) : distY*0.6;*/
		distY = (0.69*200*480)/(heigh*2.46); //in mm
		
		cout<<"distY: "<< distY<<"HEIGH: "<<heigh<<endl;
		
		walk(distY/1000,distX/2000);
				
		cout<<"distX: "<<distX<<endl;
		cout<<markers.size()<<endl;
		if(cv::waitKey(1) == 'e')
			break;
		if(angle>0)
				angle-=90;
		cout<<"angle"<<angle-90<<endl;
		rotate((angle/180)*3.14);
		break;

		/*if(distY < 50){ //Siamo sul marker
			cout<<"angle"<<angle<<endl;
			if(angle>0)
				angle-=90;
			rotate((angle/180)*3.14);
			break;
		}*/

		//moveForward(distY/2000);	
	}while(true);
 }
 


TheWalkingNao::~TheWalkingNao(void){}