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

Mat detectShadows(Mat img){
	Mat imgHSV, maskHSV;
	cvtColor(img, imgHSV, CV_BGR2HSV);
	Mat HSVbands[3];
	split(imgHSV,HSVbands);
	maskHSV = HSVbands[2] >= SHADOW_THRESH;
	return maskHSV;
}
/*
Mat imadjust(Mat img){
	double max, min, scale; 
	minMaxLoc(img,&min,&max); cout<<min<<endl; cout<<max<<endl;
	img = img-min; 
	scale = 255/(max-min);
	img = img*scale;
	return img;
}

*/

void imadjust(const Mat1b& src, Mat1b& dst, int tol = 1, Vec2i in = Vec2i(0, 255), Vec2i out = Vec2i(0, 255))
{
    // src : input CV_8UC1 image
    // dst : output CV_8UC1 imge
    // tol : tolerance, from 0 to 100.
    // in  : src image bounds
    // out : dst image buonds

    dst = src.clone();

    tol = max(0, min(100, tol));

    if (tol > 0)
    {
        // Compute in and out limits

        // Histogram
        vector<int> hist(256, 0);
        for (int r = 0; r < src.rows; ++r) {
            for (int c = 0; c < src.cols; ++c) {
                hist[src(r,c)]++;
            }
        }

        // Cumulative histogram
        vector<int> cum = hist;
        for (int i = 1; i < hist.size(); ++i) {
            cum[i] = cum[i - 1] + hist[i];
        }

        // Compute bounds
        int total = src.rows * src.cols;
        int low_bound = total * tol / 100;
        int upp_bound = total * (100-tol) / 100;
        in[0] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), low_bound));
        in[1] = distance(cum.begin(), lower_bound(cum.begin(), cum.end(), upp_bound));

    }

    // Stretching
    float scale = float(out[1] - out[0]) / float(in[1] - in[0]);
    for (int r = 0; r < dst.rows; ++r)
    {
        for (int c = 0; c < dst.cols; ++c)
        {
            int vs = max(src(r, c) - in[0], 0);
            int vd = min(int(vs * scale + 0.5f) + out[0], out[1]);
            dst(r, c) = saturate_cast<uchar>(vd);
        }
    }
}


Mat computeRationedImage(vector<Mat> bands){
	Mat1b toRet = Mat::zeros(Size(1280,960), CV_8U);
	toRet = bands[0]/bands[1] + bands[0]/bands[2] + bands[1]/bands[2] + bands[2]/bands[1] +	 bands[2]/bands[0] + bands[1]/bands[0];

	imadjust(toRet,toRet);
	imshow("adjust",toRet);
	return toRet;
}   
#if 0
Mat backgroundEstimation(Mat img){
	Mat tmp, toReturn; Size dim = img.size();
	Mat stddev, mean; vector<double> variances, means;
	int ratio = 10; double minVariance=1000;
	double minpx; double maxpx;
	Size rectDim = Size(dim.width/ratio,dim.height/ratio);
	Rect minRect;
	cvtColor(img,tmp, CV_BGR2GRAY);
	for(int r=0;r<tmp.rows;r+=rectDim.height)
		for(int c=0;c<tmp.cols;c+=rectDim.width){
			Rect rett(r,c,rectDim.width,rectDim.height);
			meanStdDev(tmp(rett),mean,stddev);
			means.push_back(mean.at<double>(0,0));
			variances.push_back(stddev.at<double>(0,0)*stddev.at<double>(0,0));
			/*Bassa varianza -> colori uniformi -> probabile sfondo*/
			double tmp = *min_element(variances.begin(),variances.end());
			if(tmp<minVariance){
				minVariance = tmp;
				minRect = rett;
			}
		}
	/*Trova il minimo (minpx) ed il massimo (maxpx) nel rettangolo con minima varianza. Dall'immagine originale mettiamo
	a 0 tutto quello appartenente a [minpx,maxpx]*/
		minMaxLoc(tmp(minRect),&minpx,&maxpx);
		toReturn = img>minpx
}
#endif
Mat backgroundEstimation(vector<Mat> bands){
	
	Mat mask, matMean,stddev; double mean, variance;
	Mat newBands[3];
	for(int i = 0; i<bands.size() ;i++){
		meanStdDev(bands[i],matMean,stddev);
		variance = stddev.at<double>(0,0);
		mean = matMean.at<double>(0,0);
		cout<<"media"<<i<<" "<<mean<<endl<<"varianza"<<i<<" "<<variance<<endl;
		bitwise_xor( bands[i] > (mean+variance/3), bands[i] < (mean-variance/3), newBands[i]);
	
	}
	bitwise_and(newBands[0],newBands[1],mask);
	bitwise_and(mask,newBands[2],mask);
	imshow("nobackmak",mask);;
	return mask;
}
//29 85 94 95 96 97
Mat applyMaskBandByBand(Mat mask, vector<Mat> bands){
	Mat toReturn;
	Mat newBands[3];
	for(int i=0;i<bands.size();i++){
		bitwise_and(mask,bands[i],newBands[i]);
	}
	merge(newBands,3,toReturn);
	return toReturn;
}

void detect(Mat img, vector<Mat>& regionsOfInterest){
	/*************INIZIALIZZAZIONI**********/
	Mat gray, hist;
	Mat out = Mat::zeros(Size(1280,960), CV_8U);
	Mat bin = Mat::zeros(Size(1280,960), CV_8U);
	Mat morph = Mat::zeros(Size(1280,960), CV_8U);
	Mat cont = Mat::zeros(Size(1280,960), CV_8U);
	Mat maskHSV = Mat::zeros(Size(1280,960), CV_8U);
	Mat rationedImage = Mat::zeros(Size(1280,960), CV_8U);
	Mat noBackMask;
	Mat tmp; vector<Mat> BGRbands;  split(img,BGRbands);
	Mat kernelEr = getStructuringElement(MORPH_ELLIPSE,Size(2,2));
	Mat kernelOp = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
	int histSize = 256; float range[] = { 0, 256 }; const float* histRange = { range };
	vector< vector<Point> > contours;
	vector<int> indexes;
	
	/***************************************/
	rationedImage = computeRationedImage(BGRbands);
	maskHSV		  = detectShadows(img);
	noBackMask    = backgroundEstimation(BGRbands);
	/*
	tmp = applyMaskBandByBand(maskHSV,BGRbands);
	split(tmp,BGRbands);
	tmp = applyMaskBandByBand(noBackMask,BGRbands);
	*/
	/*Abbassamento risoluzione radiometrica per rimozione dettagli*/
	tmp = img/255;
	tmp = tmp*240+15; //+15 per mappare il nero nel bin 15 in modo da non perdere eventuali oggettineri
	cvtColor(tmp,gray,CV_BGR2GRAY);
	gray = gray + rationedImage;
	//gray = (gray!=0)*255;
	imshow("gray",gray);
	/*Calcolo histogramma per identificazione valori di pixel significativi ( != 0 )*/
	calcHist(&gray,1,0,Mat(),hist,1,&histSize,&histRange);

	/*Calcolo gli indici corrispondenti ai bin dove c'� almeno un valore.*/
	
	for(int r=0; r<hist.rows;r++) /*Escludo bin 0 -> background****** da non escludere se dobbiamo fare detection di cose nere*/
		for(int c=0; c<hist.cols;c++)
			if(hist.at<int>(r,c)!=0)
				indexes.push_back(r);
	
	/*Estrazione componenti connesse di interesse*/
	
	for(int i=0;i<indexes.size();i++){
	
	    /*Calcolo immagine binaria del singolo bin */
		bin = gray == indexes[i];
		/*Rimozione Ombre*/
		
		bitwise_and(bin, maskHSV ,bin);
		bitwise_and(bin, noBackMask ,bin);
	
		/*Operazioni Morfologiche, kernel circolare*/
	//	morphologyEx(bin, morph, MORPH_ERODE, kernelEr, Point(-1, -1));
		morphologyEx(bin, morph, MORPH_OPEN, kernelOp, Point(-1, -1));

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
			minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
		//	Rect tmpRect(center[idx].x-boundRect[idx].width/2,center[idx].y-boundRect[idx].height/2,boundRect[idx].width,boundRect[idx].height);
			Rect tmpRect(center[idx].x-radius[idx],center[idx].y-radius[idx],radius[idx]*2,radius[idx]*2);
			//Rect tmpRect = boundRect[idx];
			Rect toPrint;
			tmpRect += Size(tmpRect.width*RECT_AUGMENT ,tmpRect.height*RECT_AUGMENT);			  // Aumenta area di RECT_ARGUMENT
			tmpRect -= Point((tmpRect.width*RECT_AUGMENT)/2 , (tmpRect.height*RECT_AUGMENT)/2 ); // Ricentra il rettangolo
			

			drawContours(cont, contours, idx, color, CV_FILLED, 8);

			if(tmpRect.x>0 && tmpRect.y>0 && tmpRect.x+tmpRect.width < tmp.cols && tmpRect.y+tmpRect.height < tmp.rows){ //Se il nuovo rettangolo allargato
																														// NON esce fuori, accettalo
				regionsOfInterest.push_back(img(tmpRect));
				toPrint = tmpRect;
			}
			else{
				toPrint = boundRect[idx];
				regionsOfInterest.push_back(img(boundRect[idx]));
			}
			rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
			circle( cont, center[idx], (int)radius[idx], color, 2, 8, 0 );

		}
		//out = out+cont;
		bitwise_xor(out,cont,out);
	}
	namedWindow("out",WINDOW_NORMAL);
	imshow("out",out);
}

