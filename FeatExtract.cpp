#include "FeatExtract.h"


FeatExtract::FeatExtract(void)
{
}

std::string FeatExtract::readMeanHueAndMoments(cv::Mat image){
	std::stringstream s;

    cv::Mat grayImage, hsvImage;
    cvtColor(image, grayImage, CV_BGR2GRAY);
    cvtColor(image, hsvImage, CV_BGR2HSV);
    float meanHue = mean(hsvImage)[0];
    cv::Moments m = moments(grayImage, false /* use true to binarize */);
    double hu[7];
    HuMoments(m, hu);
    s << meanHue / 360<<", ";
    for (int i = 0; i < 7; i++)
       s << hu[i]<<",";
	
	return s.str();
}

void FeatExtract::extract(std::vector<string> pathToDir, std::string pathToWrite, std::vector<string> types){
	 DIR *pDIR;
	 cv::Mat readed, img;
	 std::stringstream s1,s2;
	 std::string currDir,currType;
	 writer.open(pathToWrite,ios::out);
	 
     struct dirent *entry;
	 for(int i=0;i<pathToDir.size();i++){
		 currDir = pathToDir[i];
		 currType = types[i];
		 if(i==0)
			 writer<<HEADER<<"class"<<endl;
		 if( pDIR=opendir(currDir.data()) ){
            while(entry = readdir(pDIR)){
              if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 ){
				s1 <<currDir<< entry->d_name;
				std::cout<<s1.str();
				readed = cv::imread(s1.str());
				cv::resize(readed,img,cv::Size(200,200),0,0,cv::INTER_LINEAR);
				writer<<readMeanHueAndMoments(img);
				writer<<currType<<endl;
				s1.str("");
			  }
            }
                closedir(pDIR);	
        }
	}
	writer.close();
}

FeatExtract::~FeatExtract(void)
{
}
