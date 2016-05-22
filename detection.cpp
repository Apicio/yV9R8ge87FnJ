#include "detection.h"

vector<double> computeArea(vector<vector<Point> > contours){
	vector<double> toret;
	for(int i = 0; i<contours.size(); i++){
		vector<Point> row = contours.at(i);
		double area = contourArea(row);
		toret.push_back(area);
	}
	return toret;
}


void detect(Mat img, vector<Mat>& regionsOfInterest){
	/*************INIZIALIZZAZIONI**********/
	Mat gray, hist, smooth;
	Mat out = Mat::zeros(img.size(), CV_8U);
	Mat tmp = Mat::zeros(img.size(), CV_8U);
	int R=2,N=0;
	Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
	int histSize = 256; const int* c=0;
	float range[] = { 0, 256 } ;
    const float* histRange = { range };
	vector< vector<Point> > contours;

	bool discardBlack = false;
	/***************************************/
	
	/*Abbassamento risoluzione radiometrica per rimozione dettagli*/
	tmp = img/255;
	for(int r=0;r<tmp.rows;r++)
			for(int c=0;c<tmp.cols;c++ ){
				tmp.at<uchar>(r,c) = cvRound(tmp.at<uchar>(r,c));
			}
	tmp = tmp*255;
	cvtColor(tmp, gray, CV_BGR2GRAY);
	
	/*Calcolo histogramma per identificazione valori di pixel significativi ( != 0 )*/
	calcHist(&gray,1,0,Mat(),hist,1,&histSize,&histRange);
	
	/*Calcolo gli indici corrispondenti ai bin dove c'� almeno un valore.*/
	vector<int> indexes;
	for(int r=1; r<hist.rows;r++) /*Escludo bin 0 -> background****** da non escludere se dobbiamo fare detection di cose nere*/
		for(int c=0; c<hist.cols;c++)
			if(hist.at<int>(r,c)!=0)
				indexes.push_back(r);
	
	/*Estrazione componenti connesse di interesse*/
	
	for(int i=0;i<indexes.size();i++){
		Mat bin = Mat::zeros(img.size(), CV_8U);
		Mat morph = Mat::zeros(img.size(), CV_8U);
		Mat cont = Mat::zeros(img.size(), CV_8U);

		/*Calcolo immagine binaria del singolo "canale" */
		for(int r=0; r<gray.rows;r++)
			for(int c=0; c<gray.cols;c++){
				if(gray.at<uchar>(r,c)==indexes[i])
					bin.at<uchar>(r,c) = 1;
				else
					bin.at<uchar>(r,c) = 0;
			}
		
		/*Operazioni Morfologiche, kernel circolare*/
		morphologyEx(bin, morph, MORPH_ERODE, kernel, Point(-1, -1));
		morphologyEx(morph, morph, MORPH_OPEN, kernel, Point(-1, -1));

		/*Ricerca componenti connesse come meno di un certo numero di pixel*/
		
		findContours(morph, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);

		vector<double> areas = computeArea(contours);
		int max = 40000; int min = 1000;
		for(int j = areas.size()-1; j>=0; j--){
			if(areas.at(j)>max || areas.at(j)<min )
				contours.erase(contours.begin()+j);
		}

		/*Calcolo Bounding Rectangle a partire dall'immagine con componenti connesse di interesse*/
		 vector<Rect> boundRect( contours.size() );
		 vector<vector<Point> > contours_poly( contours.size() );

		/*Costruzione immagine finale ed estrazione regioni di interesse*/
		for (int idx = 0; idx < contours.size(); idx++){
			double black=0; double white=0;
			Scalar color(indexes[i]);
			approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
			boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
			drawContours(cont, contours, idx, color, CV_FILLED, 8);
			rectangle( cont, boundRect[idx].tl(), boundRect[idx].br(), color, 2, 8, 0 );
			
			/*Ulterio filtraggio delle regioni di interesse: calcoliamo pixelNONneri/pixelNeri
			********NB: SI PUO' CANCELLARE SE DOBBIAMO FARE DETECTION DI ROBE NERE **********
			*/
			Mat tmpRect = morph(boundRect[idx]);
			for(int r=0;r<tmpRect.rows && discardBlack;r++)
				for(int c=0;c<tmpRect.cols;c++){
					if(tmpRect.at<uchar>(r,c)==0) //black
						black++;
					else
						white++;
				}
			double thresh = white/(black+1); //non ci piace dividere per 0! :)
		
			/*Estrazione delle Regioni di Interesse, tramite i boundingRectangle, dall'immagine originale*/
			if(discardBlack && thresh>=0.9 )
			   regionsOfInterest.push_back(img(boundRect[idx]));
			else if (!discardBlack)
				regionsOfInterest.push_back(img(boundRect[idx]));
		}
		out = out+cont;
	}
	imshow("out",out);
}