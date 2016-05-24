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
	Mat out = Mat::zeros(Size(1280,960), CV_8U);
	Mat interpImp = Mat::zeros(Size(1280,960), CV_8U);
	Mat bin = Mat::zeros(Size(1280,960), CV_8U);
	Mat morph = Mat::zeros(Size(1280,960), CV_8U);
	Mat cont = Mat::zeros(Size(1280,960), CV_8U);
	Mat tmp;
	int R=2,N=0;
	Mat kernel = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
	int histSize = 256; const int* c=0;
	float range[] = { 0, 256 } ;
    const float* histRange = { range };
	vector< vector<Point> > contours;
	/***************************************/
	
	
	/*Interpolazione immagine per essere invarianti alla risoluzione*/
	resize(img, interpImp, interpImp.size(), 0, 0, INTER_LINEAR);

	/*Abbassamento risoluzione radiometrica per rimozione dettagli*/
	tmp = interpImp/255;
	for(int r=0;r<tmp.rows;r++)
			for(int c=0;c<tmp.cols;c++ ){
				tmp.at<uchar>(r,c) = cvRound(tmp.at<uchar>(r,c));
			}
	tmp = tmp*255;
	cvtColor(tmp, gray, CV_BGR2GRAY);
	/*Calcolo histogramma per identificazione valori di pixel significativi ( != 0 )*/
	calcHist(&gray,1,0,Mat(),hist,1,&histSize,&histRange);

	/*Calcolo gli indici corrispondenti ai bin dove c'è almeno un valore.*/
	vector<int> indexes;
	for(int r=1; r<hist.rows;r++) /*Escludo bin 0 -> background****** da non escludere se dobbiamo fare detection di cose nere*/
		for(int c=0; c<hist.cols;c++)
			if(hist.at<int>(r,c)!=0)
				indexes.push_back(r);
	
	/*Estrazione componenti connesse di interesse*/
	
	for(int i=0;i<indexes.size();i++){
		

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
		for(int j = areas.size()-1; j>=0; j--){
			if(areas.at(j)>MAX_AREA || areas.at(j)<MIN_AREA )
				contours.erase(contours.begin()+j);
		}

		/*Calcolo Bounding Rectangle a partire dall'immagine con componenti connesse di interesse*/
		 vector<Rect> boundRect( contours.size() );
		 vector<vector<Point> > contours_poly( contours.size() );
		 vector<Point2f>center( contours.size() ); 
		 vector<float>radius( contours.size() );

		/*Costruzione immagine finale ed estrazione regioni di interesse*/
		for (int idx = 0; idx < contours.size(); idx++){
			Scalar color(indexes[i]);
			approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
			boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
			Rect tmpRect = boundRect[idx];
			Rect toPrint;
			tmpRect += Size(tmpRect.width*RECT_AUGMENT ,tmpRect.height*RECT_AUGMENT);			  // Aumenta area di RECT_ARGUMENT
			tmpRect -= Point((tmpRect.width*RECT_AUGMENT)/2 , (tmpRect.height*RECT_AUGMENT)/2 ); // Ricentra il rettangolo
			
			drawContours(cont, contours, idx, color, CV_FILLED, 8);
		
			if(tmpRect.x>0 && tmpRect.y>0 && tmpRect.x+tmpRect.width < tmp.cols && tmpRect.y+tmpRect.height < tmp.rows){ //Se il nuovo rettangolo allargato
																														// NON esce fuori, accettalo
				regionsOfInterest.push_back(interpImp(tmpRect));
				toPrint = tmpRect;
			}
			else{
				toPrint = boundRect[idx];
				regionsOfInterest.push_back(interpImp(boundRect[idx]));
			}
			rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
		}
		out = out+cont;
	}
	namedWindow("out",WINDOW_NORMAL);
	imshow("out",out);
}

