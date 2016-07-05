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

void FeatExtract::extract(std::string pathToDir, std::string pathToWrite, std::string type){
	 DIR *pDIR;
	 cv::Mat readed;
	 std::stringstream s1,s2;
	 writer.open(pathToWrite,ios::out);
	 writer<<HEADER<<"class"<<endl;
        struct dirent *entry;
		if( pDIR=opendir(pathToDir.data()) ){
                while(entry = readdir(pDIR)){
                        if( strcmp(entry->d_name, ".") != 0 && strcmp(entry->d_name, "..") != 0 ){
							s1 <<pathToDir<< entry->d_name;
							std::cout<<s1.str();
							readed = cv::imread(s1.str());
							writer<<readMeanHueAndMoments(readed);
							writer<<type<<endl;
							s1.str("");
						}
                }
                closedir(pDIR);
        }

}

FeatExtract::~FeatExtract(void)
{
}
