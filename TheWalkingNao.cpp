#include "TheWalkingNao.h"

TheWalkingNao::TheWalkingNao(void)
{
}

void TheWalkingNao::ArucoFind(void)
{
    try
    {
        MarkerDetector MDetector;
        vector<Marker> Markers;
        //read the input image
        cv::Mat InImage, Sharp, grayImage;
		for(int j=1; j<40; j++){
			stringstream s;
			s << "im(13).jpg";
			InImage=cv::imread(s.str());
			InImage.copyTo(Sharp);
/*			GaussianBlur(InImage, Sharp, Size_<int>(0,0), 10);
			double alpha = 1;
			double beta = 1-alpha;
			addWeighted(InImage, alpha, Sharp, beta, 0, Sharp); */

			//Ok, let's detect
			MDetector.detect(Sharp,Markers);		
			cv::Mat croppedImage;
			//for each marker, draw info and its boundaries in the image
			for (unsigned int i=0;i<Markers.size();i++) {
				cout<<Markers[i]<<endl;								
				// Setup a rectangle to define your region of interest
				double v1[] = {Markers[i].at(0).x, Markers[i].at(1).x, Markers[i].at(2).x, Markers[i].at(3).x};
				double v2[] = {Markers[i].at(0).y, Markers[i].at(1).y, Markers[i].at(2).y, Markers[i].at(3).y};
				double xMin = fmin(v1,4);
				double xMax = fmax(v1,4);
				double yMin = fmin(v2,4);
				double yMax = fmax(v2,4);
				cv::Rect myROI(xMin, yMin, xMax-xMin, yMax-yMin);

				// Crop the full image to that image contained by the rectangle myROI
				// Note that this doesn't copy the data
				croppedImage = Sharp(myROI);
				double m1[] = {Markers[i].at(0).x-xMin, Markers[i].at(1).x-xMin, Markers[i].at(2).x-xMin, Markers[i].at(3).x-xMin};
				double m2[] = {Markers[i].at(0).y-yMin, Markers[i].at(1).y-yMin, Markers[i].at(2).y-yMin, Markers[i].at(3).y-yMin};
				Size s = croppedImage.size();
				
				for(int x = 0; x<s.height; x++){
					for(int y = 0; y<s.width; y++){
						if(!pnpoly(4,m1,m2,y,x))
							croppedImage.at<Vec3b>(x,y) = Vec3b(0,0,0);
					}
				}

//				cvtColor(croppedImage, grayImage, CV_BGR2GRAY);
//				cv::Moments m = moments(grayImage, true /* use true to binarize */);
				cout << "Direction: " << getOrientation(Markers[i]) <<"\n";

				cv::imshow("in",croppedImage);
				cv::waitKey(0);//wait for key to be pressed
				Markers[i].draw(Sharp,Scalar(0,0,255),2);
			}
				cv::imshow("full",Sharp);
				cv::waitKey(0);//wait for key to be pressed
	}
    } catch (std::exception &ex)
    {
        cout<<"Exception :"<<ex.what()<<endl;
    }
}

TheWalkingNao::~TheWalkingNao(void)
{
}