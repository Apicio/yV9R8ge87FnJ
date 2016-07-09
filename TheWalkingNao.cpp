#include "TheWalkingNao.h"

TheWalkingNao::TheWalkingNao(void)
{
}

void TheWalkingNao::ArucoFind(Mat img, double& angle){
    try{
        MarkerDetector MDetector;
        vector<Marker> Markers;
        cv::Mat Sharp, grayImage;
		Mat distorsionCoeff=cv::Mat::zeros(5,1,CV_32FC1);
		Mat cameraMatrix=cv::Mat::eye(3,3,CV_32FC1);
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
		distorsionCoeff.at<float>(4,0)=0;
		Size resolution(640,480);
		img.copyTo(Sharp);
/*		GaussianBlur(InImage, Sharp, Size_<int>(0,0), 10);
		double alpha = 1;
		double beta = 1-alpha;
		addWeighted(InImage, alpha, Sharp, beta, 0, Sharp); */
		CameraParameters camParams(cameraMatrix, distorsionCoeff, resolution);		
		MDetector.detect(Sharp,Markers,camParams,0.1,true);	
		cv::Mat croppedImage;
		//for each marker, draw info and its boundaries in the image
		for (unsigned int i=0;i<Markers.size();i++) {
		/*			
				double m1[] = {Markers[i].at(0).x-xMin, Markers[i].at(1).x-xMin, Markers[i].at(2).x-xMin, Markers[i].at(3).x-xMin};
				double m2[] = {Markers[i].at(0).y-yMin, Markers[i].at(1).y-yMin, Markers[i].at(2).y-yMin, Markers[i].at(3).y-yMin};
				Size s = croppedImage.size();
				
				for(int x = 0; x<s.height; x++){
					for(int y = 0; y<s.width; y++){
						if(!pnpoly(4,m1,m2,y,x))
							croppedImage.at<Vec3b>(x,y) = Vec3b(0,0,0);
					}
				}
				*/
				Markers[i].draw(Sharp,Scalar(0,0,255),2);
				CvDrawingUtils u;
				u.draw3dAxis(Sharp,Markers[i],camParams);
				//TODO:
				/*Aruco rileva UN solo Marker per ID, è possibile che nella stessa scena vi siano
				due marker, quindi l'idea è quella di utilizzare la posizione del centroide. Questa
				cosa è da fare se e solo se abbiamo il problema del rilevamento fra più marker*/
				if(Markers[i].id == 136)
					angle = computeAngle(Markers[i],camParams);
					
		}
				cv::imshow("full",Sharp);
    }catch (std::exception &ex){cout<<"Exception :"<<ex.what()<<endl;}
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


TheWalkingNao::~TheWalkingNao(void){}