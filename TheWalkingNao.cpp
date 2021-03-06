#include "TheWalkingNao.h"
#define NAO 1

TheWalkingNao::TheWalkingNao(){
/* Configuration */
	_ImageSharp = true;
	_SharpSigma = 20;
	_SharpThreshold = 5;
	_SharpAmount = 1;
	_medianBlur = 11;
	_markSize = 0.105;
	_invert = true;
/* Init */

	Mat distorsionCoeff=cv::Mat::zeros(5,1,CV_32FC1);
	Mat cameraMatrix=cv::Mat::eye(3,3,CV_32FC1);
/*
	cameraMatrix.at<float>(0,0)=278.236008818534;
	cameraMatrix.at<float>(0,1)=0;
	cameraMatrix.at<float>(0,2)=156.194471689706;
	cameraMatrix.at<float>(1,0)=0;
	cameraMatrix.at<float>(1,1)=279.380102992049;
	cameraMatrix.at<float>(1,2)=126.007123836447;
	cameraMatrix.at<float>(2,0)=0;
	cameraMatrix.at<float>(2,1)=0;
	cameraMatrix.at<float>(2,2)=1;
	distorsionCoeff.at<float>(0,0)=-0.0481869853715082;
	distorsionCoeff.at<float>(1,0)=0.0201858398559121;
	distorsionCoeff.at<float>(2,0)=0.0030362056699177;
	distorsionCoeff.at<float>(3,0)=-0.00172241952442813;
	distorsionCoeff.at<float>(4,0)=0;*/
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
	
	
	Size resolution(WIDTH,HEIGHT);
	CameraParameters cam(cameraMatrix, distorsionCoeff, resolution);
	camParams = cam;
}


static vector<Marker> OldMarkers;
bool sort_fun_minus(Marker a, Marker b){
	return a.getCenter().y < b.getCenter().y;
}
vector<Marker> TheWalkingNao::ArucoFind(Mat img, double& angle, bool toRemoveMarkers){
	  vector<Marker> Markers;
	  CvDrawingUtils u;
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
		}
			if(_invert){
		Mat white(img.size(), img.type(), Scalar(255,255,255));
		img = white - img;
	}
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
/*
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

*/		
				//TODO:
				/*Aruco rileva UN solo Marker per ID, � possibile che nella stessa scena vi siano
				due marker, quindi l'idea � quella di utilizzare la posizione del centroide. Questa
				cosa � da fare se e solo se abbiamo il problema del rilevamento fra pi� marker*/
				//if(Markers[i].id == 136)
			angle = computeAngle(Markers[i],camParams);
#else
			Markers[i].draw(img,Scalar(0,0,255),2);
#endif					
		}
#if NAO
		for(int i=Markers.size()-1;i>=0;i--){
			for(int k=OldMarkers.size()-1;k>=0;k--){
				Point mC = Markers[i].getCenter();
				Point mO = OldMarkers[k].getCenter();
				if(cv::norm(mC-mO) < 10)
					OldMarkers.erase (OldMarkers.begin()+k);
			}
		}
		vector<std::vector<cv::Point2f> > candidates = MDetector.getCandidates();
		vector<Marker> trackMarkers;
		vector<Point> centroids;
		for(int kk = candidates.size()-1; kk>=0 && Markers.size() !=0; kk--){
			std::vector<cv::Point2f> candidate =  candidates.at(kk);
			Point cent(0,0);
			for(int ii = 0; ii<candidate.size(); ii++){
				cent.x += candidate.at(ii).x;
				cent.y += candidate.at(ii).y;
			}
			cent.x /=4;
			cent.y /=4;
			for(int i=0;i<Markers.size();i++){
				if(cv::norm((Point)Markers[i].getCenter()-cent) < 10){
					candidates.erase (candidates.begin()+kk);
				}else{
					centroids.insert(centroids.begin(), cent);
				}
			}
		}
		for(int i=centroids.size()-1; i>=0;i--){
			int distance = INT_MAX;
			int idx = 0;
			int kdx = 0;
			for(unsigned int k=0; k<OldMarkers.size();k++){
				int eu_dist = cv::norm(centroids[i]-(Point)OldMarkers[k].getCenter());
				if(eu_dist < distance){
					idx = i;
					kdx = k;
					distance = eu_dist;
				}
			}
				if(distance < 100){
					Marker m = Marker(candidates.at(i), OldMarkers[k]);
					candidates.erase (candidates.begin()+i);
					trackMarkers.push_back(m);		
				}
		}
		// TODO: aggiornare punti, inserire nuovi marker in vettore di output in funzione della direzione
		for(int i = 0; i<trackMarkers.size(); i++)
			Markers.push_back(trackMarkers.at(i));

		std::sort(Markers.begin(), Markers.end(), sort_fun_minus);
		for(int i = 0; i<Markers.size(); i++){
			Markers[i].draw(img,Scalar(0,0,255),2);
			u.draw3dAxis(img,Markers[i],camParams);	
			cout<<"MARK  "<<Markers[i]<<endl;
		}
						

		imshow("aaaa",img);
		waitKey(700);
#endif	
	OldMarkers.clear();
	OldMarkers = Markers;		
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
    AL::ALValue val = motion->getMoveConfig("Max");
	 motion->moveTo(meters, 0, 0);//,val);//{ {"MaxStepX",0.02} , {"MaxStepY",0.101} });
 }

 void TheWalkingNao::rotate(float angle){
	 motion->post.moveTo(0, 0, angle);
 }
 void TheWalkingNao::walk(float X, float Y){
	 AL::ALValue val = motion->getMoveConfig("Default");;
  val[0][1] = 0.020; //DefX
	 val[2][1] = 0.101; //DefY
	 val[3][1] = 0.2;   //DefZ
	 val[4][1] = 0.5;   //Freq
	
	 motion->post.moveTo(X,Y,0,val);
 }
 void TheWalkingNao::infinteWalk(float velX, float velY){
	 AL::ALValue val = motion->getMoveConfig("Default");;
    val[0][1] = 0.020; //DefX
	 val[2][1] = 0.101; //DefY
	 val[3][1] = 0.2;   //DefZ
	 val[4][1] = 0.5;   //Freq
	
	 motion->post.move(velX,velY,0, val);
 }
  void TheWalkingNao::infiniteRotate(float velTheta){
	 AL::ALValue val = motion->getMoveConfig("Default");
     val[0][1] = 0.020; //DefX
	 val[2][1] = 0.101; //DefY
	 val[3][1] = 0.2;   //DefZ
	 val[4][1] = 0.5;   //Freq
	 

	 motion->post.move(0,0,velTheta,val);
 }

void TheWalkingNao::moveDownNeck(float PitchAngle){
	 const AL::ALValue jointPitch = "HeadPitch";
	 AL::ALValue stiffness = 1.0f;
     AL::ALValue time = 1.0f;
     motion->stiffnessInterpolation(jointPitch, stiffness, time);
	 AL::ALValue targetAnglePitch = PitchAngle;
     motion->angleInterpolation(jointPitch, targetAnglePitch,time, true);
}

void TheWalkingNao::moveUpNeck(){
	const AL::ALValue jointPitch = "HeadPitch";
     AL::ALValue time = 1.0f;
	 AL::ALValue targetAnglePitch = 0;
     motion->angleInterpolation(jointPitch, targetAnglePitch,time, true);
}

void TheWalkingNao::restNow() {
 	/* moves forward, without torso rotation */
 	motion->rest();
 }
bool TheWalkingNao::isMoving(){
	return motion->moveIsActive();
}





 void TheWalkingNao::moveNearMarker(Mat& img, NaoUtils nu, ALVideoDeviceProxy camProx){	
	double angle = 0;
	double distY = 0; 
	double distX = 0;
	int heigh = 0;
	vector<Marker> markers ;
	/*Becca il marker*/
	/*Vai avanti finch� non scompare salvando l'angolo*/
	/*ruota dell'angolo che hai ottenuto*/
	int k=0;
	int a=0;
	int b=0;
	int c=0;
	int d=0;
	int e=0;
	int f=0;
	int g=0;
	int h=0;
	int i=0;
	int l=0;

	char prevState = 'Z';
	char currState = 'C';
	Point center;
	do{		
		markers = ArucoFind(img,angle,false);
		
		// Posizioniamo il target al centro dell'immagine, oppure dove si trova il marker
		if(markers.size() !=0){
			k=0;
			center =(Point) markers[0].getCenter();
		}
		else{
			k++;
			if(k>3){
				center = Point(320,240);
				k=0;
			}else
				infinteWalk(0.5, 0);
		}
		int X = center.x;
		int Y = center.y; 
	   /************|******|*****|*****|*****************
	    *	75		|	   |     |	   |		75	    *
		*			|	   |	 |	   |			    *
		*			|	B  |  C  |	D  |		E		*
		*	A		|	   |     |	   |				*
		*			|  38  | 94  |	38 |				*
		*	150		|	   |     |	   |				*
		-------------------------------------------------
		*	L		| 	   |	 |	   |		F		*
		*	90	    | 	I  |  H  |	G  |		 		*
		************|******|*****|*****|*****************/
/*		bool A = X<=75 && Y<=150;
		bool B = X>75 && X<=113 && Y<=150;
		bool C = X>113 && X<=207 && Y<=150;
		bool D = X>207 && X<=245 && Y<=150;
		bool E = X>245 && X<=320 && Y<=150;
		bool F = X>245 && X<=320 && Y>150;
		bool G = X>207 && X<=245 && Y>150;
		bool H = X>113 && X<=207 && Y>150;
		bool I = X>75 && X<=113 && Y>150;
		bool L = X<=75 && Y>150;*/
		bool A = X>0 && X<=100 && Y<=336;
		bool B = X>100 && X<=180 && Y<=336;
		bool C = X>180 && X<=460 && Y<=336;
		bool D = X>460 && X<=540 && Y<=336;
		bool E = X>540 && X<=640 && Y<=336;
		bool F = X>540 && X<=640 && Y>336;
		bool G = X>460 && X<=540 && Y>336;
		bool H = X>180 && X<=460 && Y>336;
		bool I = X>100 && X<=180 && Y>336;
		bool L = X>0 && X<=100 && Y>336;
		if(C){
			c++;
			prevState = currState;
			currState = 'C';
			a=0;b=0;e=0;d=0;f=0;g=0;h=0;i=0;l = 0;
			//rectangle( img, Point(114,0),Point(207,150), Scalar(0,0,255), 2, 8, 0 ); //C
		}
		if(H){
			h++;
			prevState = currState;
			currState = 'H';
			a=0;b=0;c=0;e=0;d=0;f=0;g=0;i=0;l = 0;
			//rectangle( img, Point(113,150),Point(206,150), Scalar(0,0,255), 2, 8, 0 ); //H
		}		
		if(A){
			a++;
			prevState = currState;
			currState = 'A';
			b=0;c=0;e=0;d=0;f=0;g=0;h=0;i=0;l = 0;
			//rectangle( img, Point(0,0),Point(75,150), Scalar(0,0,255), 2, 8, 0 ); //A
		}
		if(B){
			b++;
			prevState = currState;
			currState = 'B';
			a=0;c=0;e=0;d=0;f=0;g=0;h=0;i=0;l = 0;
			//rectangle( img, Point(76,0),Point(113,150), Scalar(0,0,255), 2, 8, 0 );//B
		}
		if(L){
			l++;
			prevState = currState;
			currState = 'L';
			a=0;b=0;c=0;e=0;d=0;f=0;g=0;h=0;i=0;
			//rectangle( img, Point(0,150),Point(75,150), Scalar(0,0,255), 2, 8, 0 );		 //L
		}
		if(I){
			i++;
			prevState = currState;
			currState = 'I';
			a=0;b=0;c=0;e=0;d=0;f=0;g=0;h=0;l = 0;
			//rectangle( img, Point(75,150),Point(112,150), Scalar(0,0,255), 2, 8, 0 );    //I
		}
		if(E){
			e++;
			prevState = currState;
			currState = 'E';
			a=0;b=0;c=0;d=0;f=0;g=0;h=0;i=0;l = 0;
			//rectangle( img, Point(246,0),Point(320,150), Scalar(0,0,255), 2, 8, 0 );   //E
		}
		if(D){
			d++;
			prevState = currState;
			currState = 'D';
			a=0;b=0;c=0;e=0;f=0;g=0;h=0;i=0;l = 0;
			//rectangle( img, Point(208,0),Point(245,150), Scalar(0,0,255), 2, 8, 0 );   //D
		}
		if(F){
			f++;
			prevState = currState;
			currState = 'F';
			a=0;b=0;c=0;e=0;d=0;g=0;h=0;i=0;l = 0;
			//rectangle( img, Point(246,150),Point(320,240), Scalar(0,0,255), 2, 8, 0 ); //F
		}
		if(G){
			g++;
			prevState = currState;
			currState = 'G';
			a=0;b=0;c=0;e=0;d=0;f=0;h=0;i=0;l = 0;
			//rectangle( img, Point(207,150),Point(245,240), Scalar(0,0,255), 2, 8, 0 );		 //G
		}
		bool _event = currState != prevState;
		if(_event){
			cout<<"EVENTO: ";
			//this->motion->stopMove();
			if(C){
				cout<<"c"<<endl;
				infinteWalk(0.5, 0); // Muovi in avanti con velocit� 0.2
				// muovi avanti finch� in C
				//waitKey(1000);
			}
			if(H){
				cout<<"H"<<endl;
				walk(0.2, 0);
				// muovi in avanti di 30 cm / tunare
				//waitKey(1000);
			}		
			if(A){
				cout<<"A"<<endl;
				infiniteRotate(0.2);
				// ruota in senso antiorario finch� non B
				//waitKey(1000);
			}
			if(B){
				cout<<"B"<<endl;
				infinteWalk(0, 0.5); 
				// trasla a sinistra finch� non C
				//waitKey(1000);
			}
			if(L){
				cout<<"L"<<endl;
				infiniteRotate(0.1);
				// uguale A
				//waitKey(1000);
			}
			if(I){
				cout<<"I"<<endl;
				infinteWalk(0, 0.5);
				// uguale B
				//waitKey(1000);
			}
			if(E){
				cout<<"E"<<endl;
				infiniteRotate(-0.2);
				// ruota in senso orario finch� non D
				//waitKey(1000);
			}
			if(D){
				cout<<"D"<<endl;
				infinteWalk(0, -0.5);
				// trasla a destra finch� non C
				//waitKey(1000);
			}
			if(F){
				cout<<"F"<<endl;
				infiniteRotate(-0.2);
				// uguale a E
				//waitKey(1000);
			}
			if(G){
				cout<<"G"<<endl;
				infinteWalk(0, -0.5);
				// uguale a D
				//waitKey(1000);
			}
		}
		_event = false;
		imshow("image_nao",img);
		img = nu.see(camProx);
		}while(cv::waitKey(1)!='e');


	//	//if( markers.size() != 0){
	//	//	if(markers[0].getCenter().y > HEIGH*0.8){
	//	//		moveDownNeck(0.26);
	//	//		i++;
	//	//	}
	//	//	/*center = markers[0].getCenter();
	//	//	heigh = markers[0].getPerimeter()/4; *///supponiamo mm
	//	//	if(!isMoving())
	//	//		infinteWalk(0.2,0);
	//	//	
	//	//}
	//	//else{
	//	//	this->motion->stopMove();
	//	//	moveUpNeck();
	//	//	i=0;
	//	//	break;
	//	//}
	//	distX = markers.size() != 0 ? ( markers[0].getCenter().x - WIDTH/2 ) : distX*0.8; 
	//	distY = markers.size() != 0 ?( HEIGH - markers[0].getCenter().y) : distY*0.6;
	//	//distY = markers.size() != 0 ?(0.69*200*480)/(heigh*2.46): 0; //in mm
	////	distY =1.41e-06*std::pow((double)center.y,3) - 6.444e-05*std::pow((double)center.y,2) +0.1223*center.y + 29.88;
	////	distY = markers.size() != 0 ?-1.41e-06*std::pow((double)center.y,3)- 0.001966*std::pow((double) center.y,2) - 1.035*( center.y) + 228.7 : 0;

	//	if(!isMoving())
	//		walk(distY/100,0);
	//	if(markers.size()!=0)
	//		cout<<"YOYOYOYOYOYOOYOYOYOYOYOYOYOYOYOYO"<<endl;
	//	cout<<"distY: "<<distY<<endl;
	//	if(cv::waitKey(1) == 'e')
	//		break;
	//	if(angle>0)
	//			angle-=90;
	//	rotate((angle/180)*3.14);
	//	break;

	//	/*if(distY < 50){ //Siamo sul marker
	//		cout<<"angle"<<angle<<endl;
	//		if(angle>0)
	//			angle-=90;
	//		rotate((angle/180)*3.14);
	//		break;
	//	}*/
	//	/*distY=0;
	//	moveForward(distY/2000);	*/
	
 }
 
 


TheWalkingNao::~TheWalkingNao(void){}