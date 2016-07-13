#include "TheWalkingNao.h"
#include "detection.h"
#include <limits.h>

TheWalkingNao::TheWalkingNao(void)
{
}

void TheWalkingNao::ArucoFind(Mat img, double& angle, bool toRemoveMarkers){
    try{
        MarkerDetector MDetector;
        vector<Marker> Markers;
        cv::Mat Sharp, grayImage;
		Mat thresholded = Mat::zeros(img.size(),img.type());
		Mat toReturn = img.clone();
		vector< vector<Point> > contours;
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
		MDetector.detect(Sharp,Markers,camParams,0.15,false);	
		cv::Mat croppedImage;
		static int k = 0;
		//for each marker, draw info and its boundaries in the image
		for (unsigned int i=0;i<Markers.size();i++) {
			cout << endl << "N: " << k++ << endl;
			cvtColor(Sharp,thresholded,CV_BGR2GRAY);
			threshold(thresholded, thresholded,220,255,THRESH_BINARY);
			Size s = thresholded.size();
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
				imshow("thresh",thresholded);
				waitKey(0);
			}
			medianBlur(thresholded,thresholded,11);
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
			
			Markers[i].draw(Sharp,Scalar(0,0,255),2);
			CvDrawingUtils u;
			u.draw3dAxis(Sharp,Markers[i],camParams);
			imshow("Sharp",Sharp);
				//TODO:
				/*Aruco rileva UN solo Marker per ID, è possibile che nella stessa scena vi siano
				due marker, quindi l'idea è quella di utilizzare la posizione del centroide. Questa
				cosa è da fare se e solo se abbiamo il problema del rilevamento fra più marker*/
				if(Markers[i].id == 136)
					angle = computeAngle(Markers[i],camParams);
			
					
		}
		vector<std::vector<cv::Point2f> > cand = MDetector.getCandidates();
		for(int i=0; i<cand.size(); i++){
			MDetector.drawLine(Sharp,cand,i);
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

double TheWalkingNao::fmin(double element[], int size){
	double small = element[0];
	for(int i=0; i<size; i++){
		if (element[i] < small)
			small = element[i];
	}
	return small;
}
double TheWalkingNao::fmax(double element[], int size){
	double small = element[0];
	for(int i=0; i<size; i++){
		if (element[i] > small)
			small = element[i];
	}
	return small;
}

TheWalkingNao::~TheWalkingNao(void){}