#include "TheWalkingNao.h"
#define NAO 1
char* s_Direction[] = {"UP", "RIGHT", "DOWN", "LEFT", "STOP"};

TheWalkingNao::TheWalkingNao(){
	/* Configuration */
	_ImageSharp = true;
	_SharpSigma = 20;
	_SharpThreshold = 5;
	_SharpAmount = 3;
	_medianBlur = 11;
	_markSize = 0.105;
	_invert = true;
	/* Init */
	Mat distorsionCoeff=cv::Mat::zeros(5,1,CV_32FC1);
	Mat cameraMatrix=cv::Mat::eye(3,3,CV_32FC1);/*
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
												*/
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
	/*	Mat distorsionCoeff=cv::Mat::zeros(5,1,CV_32FC1);
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
	distorsionCoeff.at<float>(4,0)=3.0809776264102879e+002;*/
	Size resolution(WIDTH,HEIGHT);
	CameraParameters cam(cameraMatrix, distorsionCoeff, resolution);
	camParams = cam;
}


static vector<Marker> OldMarkers;

bool sort_fun_minus(Marker a, Marker b){
	return a.getCenter().y < b.getCenter().y;
}
vector<Marker> TheWalkingNao::ArucoFind(Mat img, double& angle, bool toRemoveMarkers){
	Mat mask1, mask2, origG, invertG;
	static int i = 0;
	i++;
	stringstream ss;
	ss << "take\\image" << i <<".jpg";
	imwrite(ss.str(),img);
	if(_invert){
		Mat white(img.size(), img.type(), Scalar(255,255,255));
		img.copyTo(origG);
		img = white - img;
		ss.str("");
		ss << "take\\inv\\image" << i <<".jpg";
		imwrite(ss.str(),img);
		cvtColor(img,invertG,CV_BGR2GRAY);
		cvtColor(origG,origG,CV_BGR2GRAY);
		threshold(invertG,mask1,30,255,THRESH_BINARY_INV);
		threshold(origG,mask2,225,255,THRESH_BINARY);
		bitwise_and(mask1,mask2, mask1);
		imshow("aaaa", mask1);
		ss.str("");
		ss << "take\\mask\\image" << i <<".jpg";
		imwrite(ss.str(),mask1);	
	}

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
			Markers[i].draw(img,Scalar(0,0,255),2);

			u.draw3dAxis(img,Markers[i],camParams);		
			cout<<"MARK  "<<Markers[i]<<endl;
			//TODO:
			/*Aruco rileva UN solo Marker per ID, è possibile che nella stessa scena vi siano
			due marker, quindi l'idea è quella di utilizzare la posizione del centroide. Questa
			cosa è da fare se e solo se abbiamo il problema del rilevamento fra più marker*/
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
				if(cv::norm(mC-mO) < 100)
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
				if(cv::norm((Point)Markers[i].getCenter()-cent) < 100){
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
				Marker m = Marker(candidates.at(idx), OldMarkers[kdx]);
				candidates.erase (candidates.begin()+idx);
				trackMarkers.push_back(m);		
			}
		}
		// TODO: aggiornare punti, inserire nuovi marker in vettore di output in funzione della direzione
		//		imshow("VERI",img);

		for(int i = 0; i<trackMarkers.size(); i++)
			Markers.push_back(trackMarkers.at(i));

		std::sort(Markers.begin(), Markers.end(), sort_fun_minus);
		for(int i = 0; i<Markers.size(); i++){
			Markers[i].draw(img,Scalar(0,0,255),2);
			u.draw3dAxis(img,Markers[i],camParams);	
		}

		for(int i = 0; i<candidates.size(); i++)
			MDetector.drawLine(img, candidates, i);

		//		imshow("AGGIUNTI",img);

		CBlobResult blobs = CBlobResult( mask1 ,Mat(),8);
		int bmax = 200;
		int bmin = 30;
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetLength(), B_GREATER, bmax );
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetLength(), B_LESS, bmin );
		mask1.setTo(0);
		for(int i=0;i<blobs.GetNumBlobs();i++){
			CBlob blob = blobs.GetBlob(i);
			Point center = Point(blob.MinX() + (( blob.MaxX() - blob.MinX() ) / 2.0), blob.MinY() + (( blob.MaxY() - blob.MinY() ) / 2.0));
			double angle =  blob.GetEllipse().angle;
			CvRect rect = blob.GetBoundingBox();
			//for(int k=0; k<Markers.size(); k++){
			//
			//			}
			blobs.GetBlob(i)->FillBlob(mask1,CV_RGB(255,255,255),0,0,true);
		}
		//		imshow("masked",mask1);
		//GetBoundingBox
		//CBlobGetOrientation


#endif	
		//OldMarkers.clear();
		for(int i = 0; i<Markers.size(); i++)
			OldMarkers.push_back(Markers.at(i));	
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

Mat getRotatedRoi(Mat& image, RotatedRect rect, float angle) {
	Mat M, src, rotated, cropped;
	Size rect_size = rect.size;
	if (rect.angle < -45.) {
		angle += 90.0;
		swap(rect_size.width, rect_size.height);
	}
	M = getRotationMatrix2D(rect.center, angle, 1.0);
	warpAffine(image, rotated, M, image.size(), INTER_CUBIC);
	getRectSubPix(rotated, rect_size, rect.center, cropped);
	return cropped;
}

Mat getRotatedRoi(Mat& image, RotatedRect rect) {
	return getRotatedRoi(image, rect, rect.angle);
}

bool _sortFun(MarkersInfo a, MarkersInfo b){
	Point center(0.5*a.img_size.width,b.img_size.height);
	return norm(a.center-center) < norm(b.center-center);
}
bool TheWalkingNao::pathfinder(Mat img, Direction& direction, Point& center){
	if(_ImageSharp){
		Mat blurred; 
		GaussianBlur(img, blurred, Size(), _SharpSigma, _SharpSigma);
		Mat lowContrastMask = abs(img - blurred) < _SharpThreshold;
		Mat sharpened = img*(1+_SharpAmount) + blurred*(-_SharpAmount);
		img.copyTo(sharpened, lowContrastMask);
		sharpened.copyTo(img);
	}
	/* Dichiarazione */
	Mat imgHSV;
	vector<Mat> bands;
	vector<Mat> bandsHSV;
	cvtColor(img,imgHSV, CV_BGR2HSV);
	split(imgHSV,bandsHSV);
	split(img,bands);
	vector<MarkersInfo> _markers;
	Mat gray, dest; cvtColor(img,gray,CV_BGR2GRAY);
	double _minSize = 0.05;
	double _maxSize = 0.5;
	double minSize=_minSize*max(img.cols,img.rows)*4;
	double maxSize=_maxSize*max(img.cols,img.rows)*4;
	Mat res = Mat(img.size(), img.type()); res.setTo(255);
	/* Detect Blob, filtraggio con OTSU */
	int otsuTRGB = getThreshVal_Otsu_8u(bands[2])+20;
	Mat mask1 = bandsHSV[0] > 35/2;
	Mat mask2 = bandsHSV[0] < 325/2;
	Mat maskTOT; bitwise_and(mask1,mask2, maskTOT);
	bitwise_not(maskTOT,maskTOT);
	bitwise_and(maskTOT,bands[2], bands[2]);
	std::vector<cv::Point> approxCurve;
	do{
		threshold(bands[2],dest,otsuTRGB,255,THRESH_BINARY);
		otsuTRGB += 5;
		// TODO miglirare detection dei mark vengono visti male.
	}while(countNonZero(dest)>(0.145*gray.rows*gray.cols) & otsuTRGB<=255);
	imshow("otsu",dest);
	/* Filtraggio dimensione del blob */
	CBlobResult blobs = CBlobResult( dest ,Mat(),4);
	blobs.Filter( blobs, B_EXCLUDE, CBlobGetLength(), B_GREATER, 1000 );
	blobs.Filter( blobs, B_EXCLUDE, CBlobGetLength(), B_LESS, 60 );
	/* Filtraggio per numero di lati */
	for(int i=0;i<blobs.GetNumBlobs();i++){
		double p2 = blobs.GetBlob(i)->Perimeter();
		std::vector<vector<cv::Point> > quadrato = blobs.GetBlob(i)->GetExternalContour()->GetContours();
		std::vector<cv::Point> curve; 
		convexHull(quadrato.at(0), curve);
		approxPolyDP(curve, curve, p2*0.04, true);
		Mat deb = Mat(img.size(), img.type()); res.setTo(255);;
		blobs.GetBlob(i)->FillBlob(deb,CV_RGB(255,255,255),0,0,true);
		if(curve.size()==4 || curve.size()==5){
			CvRect rect = blobs.GetBlob(i)->GetBoundingBox();
			//double area = blobs.GetBlob(i)->Area();
			//double lato = (area/p2)*4;
			double ratio = double(rect.width)/double(rect.height);
			//if(ratio>= 0.90 && ratio <= 1.1){
			blobs.GetBlob(i)->FillBlob(res,CV_RGB(255,255,255),0,0,true);
			MarkersInfo mk;
			mk.img_size = img.size();
			Mat rect_im = res(rect);
			Mat white(rect_im.size(), rect_im.type(), Scalar(255,255,255));	
			mk.marker = white-rect_im;
			mk.rect = rect;
			mk.center = blobs.GetBlob(i)->getCenter();
			_markers.push_back(mk);
		}
	}
	/* Filtro per numero di elementi interni ad un quadrato, se c'è ne'è solo uno abbiamo un oggetto di interesse */
	//	vector<vector<Point > > cont_marker;
	for(int i = _markers.size()-1; i>=0; i--){
		vector<vector<Point > > tmp_cont_marker;
		Mat gray;
		cvtColor(_markers[i].marker,gray,CV_BGR2GRAY);
		Mat kernelEr = getStructuringElement(MORPH_RECT,Size(7,7));
		dilate(gray,gray,kernelEr);	
		erode(gray,gray,kernelEr);
		findContours(gray.clone(),tmp_cont_marker,CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
		if(tmp_cont_marker.size() != 1 ){
			_markers[i].isValid = false;
		}
		else{
			/*					for(int c = 0; c<tmp_cont_marker.size(); c++){
			_markers[i].contour.insert(_markers[i].contour.end(), tmp_cont_marker[c].begin(), tmp_cont_marker[c].end());
			}*/
			gray.copyTo(_markers[i].marker);
			_markers[i].isValid = true;
			_markers[i].contour = tmp_cont_marker[0];
		}
	}
	/* Se sono rimasti marker individuiamoli */
	for(int i = 0; i<_markers.size(); i++){
		if(!_markers[i].isValid) continue;
		Mat gray = _markers[i].marker;
		CBlobResult new_blobs = CBlobResult(gray ,Mat(),4);
		double area = new_blobs.GetBlob(0)->Area();
		//		imshow("prima",gray);
		//			waitKey(0);
		if(area > 400){		
			RotatedRect rrect = minAreaRect(_markers[i].contour);
			Mat rroi = getRotatedRoi(gray, rrect);
			/*			imshow("dopo",gray);
			waitKey(0);
			imshow("der",rroi);
			waitKey(0);*/
			resize(rroi,rroi,Size(400,400));
			double swidth=rroi.rows/2;
			double w_density[2][2];
			for (int _i=0; _i<2; _i++)
			{	
				for (int _j = 0; _j<2; _j++)
				{
					int Xstart=(_j)*(swidth);
					int Ystart=(_i)*(swidth);
					Mat square=rroi(Rect(Xstart,Ystart,swidth,swidth));
					double numWhite = countNonZero(square);
					w_density[_i][_j] = numWhite/(rroi.rows*rroi.rows/4);
				}
			}
			double up = w_density[0][0] + w_density[0][1];
			double right = w_density[0][1] + w_density[1][1];
			double left = w_density[0][0] + w_density[1][0];
			double down =  w_density[1][1] + w_density[1][0];
			Direction angle = STOP;
			double marker_ratio;
			angle = (up>right && up>left && up>down)? UP:angle;
			angle = (right>up && right>left && right>down)? RIGHT:angle;
			angle = (left>right && left>up && left>down)? LEFT:angle;
			angle = (down>right && down>left && down>up)? DOWN:angle;
			std::vector<cv::Point> curve; 
			convexHull(_markers[i].contour, curve);
			double p2 = arcLength(_markers[i].contour, true);
			approxPolyDP(_markers[i].contour, curve, p2*0.04, true);
			if(curve.size()==3)
				angle = STOP;
			curve.clear();
			_markers[i].dir = angle;
			_markers[i].isValid = true;
		}else{
			_markers[i].isValid = false;
		}
	}

	bool foundSomethingIWantToShow = false;
	int dist = INT_MAX;
	std::sort(_markers.begin(),_markers.end(),_sortFun);
	for(int ii = 0; ii<_markers.size(); ii++){
		cout << _markers[ii].dir << endl;
	}
	for(int ii = 0; ii<_markers.size(); ii++){
		if(!_markers[ii].isValid) continue;
		//if(dist>_markers[ii].center.y /*&& _markers.center.y < soglia */){
		dist = _markers[ii].center.y;
		direction = _markers[ii].dir;
		center = _markers[ii].center;
		foundSomethingIWantToShow = true;
		break;
		//}
	}
	return foundSomethingIWantToShow;
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
void TheWalkingNao::walk(float X, float Y, float angle){
	AL::ALValue val = motion->getMoveConfig("Default");
	val[0][1] = 0.06;
	val[3][1] = 0.5;   //Freq

	//	 val[0][1] = 0.020; //DefX
	//	 val[2][1] = 0.101; //DefY
	//	 val[3][1] = 0.2;   //DefZ
	//	 val[4][1] = 0.5;   //Freq

	motion->post.moveTo(X,Y,angle,val);
}
void TheWalkingNao::infiniteWalk(float velX, float velY, float angle){
	AL::ALValue val = motion->getMoveConfig("Default");;
	val[0][1] = 0.06;
	val[3][1] = 0.5;   //Freq

	motion->post.move(velX,velY,angle, val);
}
void TheWalkingNao::infiniteRotate(float velTheta){
	AL::ALValue val = motion->getMoveConfig("Default");
	val[0][1] = 0.020; //DefX
	val[2][1] = 0.101; //DefY
	val[3][1] = 0.2;   //DefZ
	val[4][1] = 0.5;   //Freq
	motion->post.move(0,0,velTheta,val);
}

void TheWalkingNao::markerExplore(ALVideoDeviceProxy camProx, NaoUtils nu){
	const AL::ALValue jointYaw = "HeadYaw";
	this->motion->waitUntilMoveIsFinished();
	this->robotPosture->goToPosture("Stand",0.5);

	bool directions[3]; int i;
	try {
		AL::ALValue stiffness = 1.0f;
		AL::ALValue time = 1.0f;
		this->motion->stiffnessInterpolation(jointYaw, stiffness, time);

		float orders[] = {0,-0.3f,0.6f};
		for(i=0;i<3;i++){	
			AL::ALValue targetAnglesYaw= AL::ALValue::array(orders[i]);
			AL::ALValue targetTimesYaw= AL::ALValue::array(1.0f);
			bool isAbsolute = true;
			this->motion->angleInterpolation(jointYaw, targetAnglesYaw, targetTimesYaw, isAbsolute);
			this->motion->waitUntilMoveIsFinished();
			cv::Mat img = nu.see(camProx);
			imshow("explore",img);
			waitKey(0);
			Direction d; Point p;
			if(pathfinder(img,d,p))
				break;

		}
		this->robotPosture->goToPosture("StandInit",0.5);
		cout<<"ENTRO NEL CASE: "<<i<<endl;
		switch(i){
		case 1:
			walk(0.2,0,-3.14/3);
			this->motion->waitUntilMoveIsFinished();
			break;
		case 2:
			walk(0.2,0,3.14/3);
			this->motion->waitUntilMoveIsFinished();
			break;
		case 0:
			walk(0.2,0,0);
			this->motion->waitUntilMoveIsFinished();
			break;	
		default:
			walk(0.1,0,0);
			this->motion->waitUntilMoveIsFinished();
			markerExplore(camProx, nu);
			break;	
		}
	}catch(std::exception& e){cout<<e.what()<<endl;}
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

void TheWalkingNao::moveNearMarker(Mat& img, NaoUtils nu, ALVideoDeviceProxy camProx){	
	Size img_size = img.size();
	double angle = 0;
	double distY = 0; 
	double distX = 0;
	int heigh = 0; 
	int k=0;
	vector<Marker> markers ;
	char prevState = 'Z';
	char currState = 'C';
	Point center;
	/*
	infiniteWalk(WSPEED, 0,0);
	while(cv::waitKey(1)!='e'){

	static int i = 0;
	i++;
	stringstream ss;
	ss << "_new\\leo\\casa_1" << i <<".jpg";
	imwrite(ss.str(),img);
	img = nu.see(camProx);

	}*/
	double x100 = 0.35;
	Direction moveTo = STOP;
	Point s(img_size.width*0.5,img_size.height*0.5);
	bool doingAction = false;
	do{	
		bool _event = pathfinder(img, moveTo, s);
		if(_event){
			bool A = s.x<=x100*img_size.width;
			bool E = s.x>=(1-x100)*img_size.width;
			if(A | E){
				if(A){
					cout << "ALLIGN LEFT" << endl;
					walk(0, 0.08, 0);
					this->motion->waitUntilMoveIsFinished();
				}
				if(E){
					cout << "ALLIGN RIGHT" << endl;
					walk(0, -0.08, 0);
					this->motion->waitUntilMoveIsFinished();
				}
			}else{
				if(moveTo == UP){
					cout << "GOING" << endl;
					walk(0.5, 0, 0);
					this->motion->waitUntilMoveIsFinished();
				}
				if(moveTo == RIGHT){
					cout << "RIGHT" << endl;
					walk(0.65, 0.05, -75*3.14/180);
					this->motion->waitUntilMoveIsFinished();
				}
				if(moveTo == LEFT){
					cout << "LEFT" << endl;
					walk(0.65, -0.05, 75*3.14/180);
					this->motion->waitUntilMoveIsFinished();
				}
				if(moveTo == STOP){
					cout << "STOP" << endl;
					walk(0.5, 0, 0);
					this->motion->waitUntilMoveIsFinished();
					break;
				}
			}
		}
		cout << "esco" << endl;
		img = nu.see(camProx);
		imshow("naosee",img);
	}while(cv::waitKey(1)!='e');
#if 0
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
				center = Point(160,240);
			}
		}
		static int oX = X;
		static int oY = X;
		static int X = center.y;
		static int Y = center.x; 

		Direction _dir;
		if(oX < X)
			_dir = RIGHT;
		else
			_dir = LEFT;

		cout<<center<<endl;
		/************|******|*****|*****|*****************
		*	75		|	   |     |	   |		75	    *
		*			|	   |	 |	   |			    *
		*			|	   |  C  |	   |		E		*
		*	A		|	   |     |	   |				*
		*			|  38  | 94  |	38 |				*
		*	150		|	   |     |	   |				*
		-------------------------------------------------
		*	L		| 	   |	 |	   |		F		*
		*	90	    | 	I  |  H  |	G  |		 		*
		************|******|*****|*****|*****************/
		/*		bool A = X>0 && X<=100 && Y<=336;
		bool C = X>180 && X<=460 && Y<=336;
		bool E = X>540 && X<=640 && Y<=336;
		*/
		bool A = X<=95;
		bool C = X>95 && X<225;
		bool E = X>=225;
		//cout << "Curr "<< currState "Prev" << prevState << endl;


		/*		bool A = X>0 && X<=100 && Y<=336;
		bool B = X>100 && X<=180 && Y<=336;
		bool C = X>180 && X<=460 && Y<=336;
		bool D = X>460 && X<=540 && Y<=336;
		bool E = X>540 && X<=640 && Y<=336;
		bool F = X>540 && X<=640 && Y>336;
		bool G = X>460 && X<=540 && Y>336;
		bool H = X>180 && X<=460 && Y>336;
		bool I = X>100 && X<=180 && Y>336;
		bool L = X>0 && X<=100 && Y>336;*/
		if(C){		
			prevState = currState;
			currState = 'C';		
			//rectangle( img, Point(114,0),Point(207,150), Scalar(0,0,255), 2, 8, 0 ); //C
		}
		/*	if(H){

		prevState = currState;
		currState = 'H';

		//rectangle( img, Point(113,150),Point(206,150), Scalar(0,0,255), 2, 8, 0 ); //H
		}	*/	
		if(A && markers.size() !=0){		
			prevState = currState;
			currState = 'A';
			//rectangle( img, Point(0,0),Point(75,150), Scalar(0,0,255), 2, 8, 0 ); //A
		}
		/*		if(B){
		prevState = currState;
		currState = 'B';
		}*/
		/*if(L){

		prevState = currState;
		currState = 'L';
		//rectangle( img, Point(0,150),Point(75,150), Scalar(0,0,255), 2, 8, 0 );		 //L
		}*/
		/*		if(I){

		prevState = currState;
		currState = 'I';
		}*/
		if(E && markers.size() !=0){			
			prevState = currState;
			currState = 'E';
			//rectangle( img, Point(246,0),Point(320,150), Scalar(0,0,255), 2, 8, 0 );   //E
		}
		/*		if(D){

		prevState = currState;
		currState = 'D';
		}*/
		/*if(F){

		prevState = currState;
		currState = 'F';
		//rectangle( img, Point(246,150),Point(320,240), Scalar(0,0,255), 2, 8, 0 ); //F
		}*/
		/*		if(G){

		prevState = currState;
		currState = 'G';
		}*/
		bool _event = currState != prevState || k == 0;
		if(_event){
			cout<<"EVENTO: ";
			//this->motion->stopMove();
			if(C){
				cout<<"C"<<endl;
				infiniteWalk(WSPEED, 0,0); // Muovi in avanti con velocità 0.2
				// muovi avanti finchè in C
				//waitKey(1000);
			}
			/*			if(H){
			cout<<"H"<<endl;
			int localAngle = 0;
			if(angle < 45 || angle>-45)
			localAngle = -90;
			if(angle >= 45 || angle<135)
			localAngle = 0;
			if(angle >= 135 || angle<-135)
			localAngle = 90;
			walk(WDIST, 0, localAngle);
			// muovi in avanti di 30 cm / tunare
			//waitKey(1000);
			}*/		
			if(A){
				cout<<"A"<<endl;
				infiniteWalk(0.5*WSPEED,WSPEED,0);
				// ruota in senso antiorario finchè non B
				//waitKey(1000);
			}
			/*			if(B){
			cout<<"B"<<endl;
			infiniteWalk(0.5*WSPEED, 0.4*WSPEED,0); 
			}*/
			if(E){
				cout<<"E"<<endl;
				infiniteWalk(0.5*WSPEED,-WSPEED,0);
				// ruota in senso orario finchè non D
				//waitKey(1000);
			}
			/*			if(D){
			cout<<"D"<<endl;
			infiniteWalk(0.5*WSPEED, -0.4*WSPEED,0);
			}*/
			/*			if(F){
			cout<<"F"<<endl;
			infiniteWalk(0,-WSPEED,0);
			}*/
			/*			if(G && ! G){
			cout<<"G"<<endl;
			infiniteWalk(0, -WSPEED,0);
			}*/
			/*			if(L){
			cout<<"L"<<endl;
			infiniteWalk(0,WSPEED,0);
			}*/
			/*			if(I){
			cout<<"I"<<endl;
			infiniteWalk(0, WSPEED,0);
			}*/
		}
		if(C & Y<120){
			int localAngle = 0;
			if(angle < 45 || angle>-45)
				localAngle = -90;
			if(angle >= 45 || angle<135)
				localAngle = 0;
			if(angle >= 135 || angle<-135)
				localAngle = 90;
			walk(0.5, 0.1*localAngle/90, localAngle*180/3.14);
			while(isMoving()){};
		}
		_event = false;
		//imshow("image_nao",img);
		img = nu.see(camProx);
	}while(cv::waitKey(1)!='e');

#endif
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