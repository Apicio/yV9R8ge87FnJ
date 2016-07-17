#include "FeatExtract.h"
#include "detection.h"

FeatExtract::FeatExtract(void)
{}


double FeatExtract::Log2( double n )  
{  
    // log(n)/log(2) is log2.  
    return std::log( n ) / std::log( 2.0 );  
	
}

std::string FeatExtract::readMeanHueAndMoments(cv::Mat image){
	std::stringstream s;
    cv::Mat grayImage, hsvImage;
   
    cvtColor(image, hsvImage, CV_BGR2HSV);
    float meanHue = mean(hsvImage)[0];
    cv::resize(image,image,cv::Size(200,200),0,0,cv::INTER_LINEAR);
				/*if(i%10==0)
					imshow(s1.str(),readed);*/
    cvtColor(image, grayImage, CV_BGR2GRAY);
    cv::Moments m = moments(grayImage, false /* use true to binarize */);
    double hu[7];
    HuMoments(m, hu);
    s << meanHue / 360<<", ";
    for (int i = 0; i < 7; i++)
       s << hu[i]<<",";
	return s.str();
}

std::string FeatExtract::readStdDevHue(cv::Mat image){
	std::stringstream s;
    cv::Mat grayImage, hsvImage, pad, stdDevHue;
	std::vector<Mat> hsvBands; 
    cvtColor(image, hsvImage, CV_BGR2HSV);
	split(hsvImage,hsvBands);
	meanStdDev(hsvBands[0],pad,stdDevHue);
	s<<stdDevHue.at<double>(0,0)<<",";
	return s.str();
}

/*NB:USARE QUESTA DURANTE LA MOVIMENTAZIONE
L'idea è che la detection ritorna la struttura Blob popolata con tutte le informazioni utili all'estrazione delle features. Ovviamente
noi dobbiamo estrarre le features da ogni singolo oggetto che abbiamo estratto, quindi currIdx serve per selezionare l'iesimo oggetto.
*/
std::string FeatExtract::extractDuringMovement(Blob b, bool toMask){
	cv::Mat img =(b.cuttedWithBack).clone();
	cv::Mat mask;
	vector<Mat> bands;
	std::stringstream s1;
	 if(toMask)
		 img = b.cuttedImages.clone();
	  imshow("maskd", img);
	   waitKey(0);
	s1<<readMeanHueAndMoments(img);
	s1<<readStdDevHue(img);
	s1<<computeEntropy(img)<<",";
	s1<<b.area<<","<<b.distance<<",";
	s1<<"?"; //per Weka
	return s1.str();
}

double FeatExtract::computeEntropy(cv::Mat image){
	Mat histogram,gray;   int histSize = 256; // # di bin.
	gray = image.clone(); float sym_occur;
	cvtColor(gray,gray,CV_BGR2GRAY);
    float range[] ={0,255}; const float *ranges[] = {range};
    calcHist(&gray, 1,0, Mat(), histogram,1, &histSize,ranges,true,false);

	int cnt = 0;
	double entr = 0;
	float total_size = image.rows * image.cols; //total size of all symbols in an image
	
	for (int i = 0;i<histSize;i++)
	{
		 sym_occur = histogram.at<float>(i, 0); //the number of times a sybmol has occured
		if (sym_occur>0 && total_size>0) //log of zero goes to infinity
		{
			entr += (sym_occur / total_size)*(Log2(total_size / sym_occur));
		}
	}
	return entr;
}

/*double FeatExtract::readBboxComparasion(cv::Mat image){
	double rows = image.rows;
	double cols = image.cols;
	return rows/cols;
}*/


void FeatExtract::extract(std::vector<string> pathToDir, std::string pathToWrite, std::vector<string> types, bool toMask){
	 DIR *pDIR;
	 cv::Mat readed, img,mask;
	 vector<Mat> bands;
	 std::stringstream s1,s2;
	 std::string currDir,currType;
	 std::ifstream reader;
	 char infos[100];
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
				if(s1.str().find(".jpg")!=std::string::npos){
					readed = cv::imread(s1.str());
					if(toMask){
						mask = backgroundRemoval(readed);
						split(readed,bands);
						readed = applyMaskBandByBand(mask,bands);
					}

				/*CALCOLO RAPPORTO FRA BBOX, NB: PRIMA DELLA RESIZE!!!*/
			//	writer<<readBboxComparasion(readed)<<",";

					writer<<readMeanHueAndMoments(img);
					writer<<readStdDevHue(img);
					writer<<computeEntropy(img)<<",";
					
				}else{
					reader.open(s1.str(),ios::in);
					string tmp;
					getline(reader,tmp);
					writer<<tmp<<",";
					writer<<currType<<endl;
					reader.clear();
					reader.close();
				}
				
				
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
