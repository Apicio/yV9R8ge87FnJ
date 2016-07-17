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

Point computeCentroid(vector<Point> contour){
	Point toReturn; int x=0; int y=0;
	for(int i=0;i<contour.size();i++){
		x+=contour[i].x;
		y+=contour[i].y;
	}
	x/=contour.size(); y/=contour.size();
	toReturn.x=x; toReturn.y=y;
	return toReturn;
}



Mat detectShadows(Mat img){
	Mat imgHSV, maskHSV;
	cvtColor(img, imgHSV, CV_BGR2HSV);
	Mat HSVbands[3];
	split(imgHSV,HSVbands);
	maskHSV = HSVbands[2] >= SHADOW_THRESH;
	return maskHSV;
}

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
	Mat1b toRet = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	toRet = bands[0]/bands[1] + bands[0]/bands[2] + bands[1]/bands[2] + bands[2]/bands[1] +	 bands[2]/bands[0] + bands[1]/bands[0];
	imadjust(toRet,toRet);
	return toRet;
}   

Mat backgroundRemoval(Mat& img){
	Mat imgHSV; Mat HSVbands[3]; Mat toRet = img.clone(); Mat mask1,mask2,maskTOT;
	cvtColor(img,imgHSV,CV_BGR2HSV);
	split(imgHSV,HSVbands);
	mask1 = HSVbands[0] <= 90/2;
	mask2 = HSVbands[0] >= 270/2;
	//bitwise_and(mask1,mask2,maskTOT);
	maskTOT = mask1 + mask2;
	return maskTOT;

}

Mat applyMaskBandByBand(Mat mask, vector<Mat> bands){
	Mat toReturn;
	Mat newBands[3];
	for(int i=0;i<bands.size();i++){
		bitwise_and(mask,bands[i],newBands[i]);
	}
	merge(newBands,3,toReturn);
	return toReturn;
}

Mat bwareaopen(Mat& img, int size)
 {
     CBlobResult blobs;
     blobs = CBlobResult( img ,Mat(),4);
     blobs.Filter( blobs, B_INCLUDE, CBlobGetLength(), B_GREATER, size );

     Mat newimg(img.size(),img.type());
     newimg.setTo(0);
     for(int i=0;i<blobs.GetNumBlobs();i++)
     {
         blobs.GetBlob(i)->FillBlob(newimg,CV_RGB(255,255,255),0,0,true);
     }
     return newimg;
 }

Mat removeUseless(Mat& img, int low, int hight)
 {
     CBlobResult blobs;
     blobs = CBlobResult( img ,Mat(),4);
	 blobs.Filter( blobs, B_OUTSIDE, CBlobGetLength(), low, hight );

     Mat newimg(img.size(),img.type());
     newimg.setTo(0);
     for(int i=0;i<blobs.GetNumBlobs();i++)
     {
         blobs.GetBlob(i)->FillBlob(newimg,CV_RGB(255,255,255),0,0,true);
     }
     return newimg;
 }

Mat computeWhiteMaskLight(Mat& input){
	Mat img, gray;
	input.clone().convertTo(img,CV_64F);
	Mat BGRbands[3] = {Mat::zeros(img.size(), CV_64F), Mat::zeros(img.size(), CV_64F),Mat::zeros(img.size(), CV_64F)};  
	Mat I1 = Mat::zeros(img.size(), CV_64F);
	Mat I2 = Mat::zeros(img.size(), CV_64F);
	Mat I3 = Mat::zeros(img.size(), CV_64F);
	Mat Id1 = Mat::zeros(img.size(), CV_64F);
	Mat Id2 = Mat::zeros(img.size(), CV_64F);
	Mat Id3 = Mat::zeros(img.size(), CV_64F);
	Mat I = Mat::zeros(img.size(), CV_64F);
	Mat mask = Mat::zeros(img.size(), CV_8U);
	vector< vector<Point> > contours;
	Mat mask2 = Mat::zeros(img.size(), CV_8U);
	split(img,BGRbands);
	I1 = BGRbands[0]-BGRbands[1];
	I1 = I1.mul(I1);
	I2 = BGRbands[0]-BGRbands[2];
	I2 = I2.mul(I2);
	I3 = BGRbands[1]-BGRbands[2];
	I3 = I3.mul(I3);
	Id1 = I1-I2;
	Id2 = I1-I3;
	Id3 = I3-I2;
	Id1 = Id1.mul(Id1);
	Id2 = Id2.mul(Id2);
	Id3 = Id3.mul(Id3);
	I = Id1+Id2+Id3;
	sqrt(I,I);
	sqrt(I,I);
	I.convertTo(mask,CV_8U);
	threshold(mask,mask,50,255,THRESH_BINARY_INV);
	Mat d = detectShadows(input);
	bitwise_and(mask,d,mask);
	cvtColor(input,gray,CV_BGR2GRAY);
	threshold(gray,mask2,160,255,THRESH_BINARY);
	mask = mask+mask2;
	medianBlur(mask,mask,9);
	findContours(mask, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	vector<double> areas = computeArea(contours);
	for(int j = areas.size()-1; j>=0; j--){
		if(areas.at(j)>10000 || areas.at(j)<400 ) // 10000 400
			contours.erase(contours.begin()+j);
	}
	Mat out = Mat::zeros(Size(img.cols,img.rows), CV_8U);
	for (int idx = 0; idx < contours.size(); idx++)
		drawContours(out, contours, idx, Scalar(255,255,255), CV_FILLED, 8);
	return out;
}


Mat computeWhiteMaskShadow(Mat& img)
 {
    // I = rgbFrame(:,:,1)>80 & rgbFrame(:,:,3)>80 | rgbFrame(:,:,3)>80 & abs(double(rgbFrame(:,:,1))-double(rgbFrame(:,:,3)))<20;
    // I = medfilt2(I,[20,20]);
	Mat BGRbands[3];  
	split(img,BGRbands);
	Mat maskB, maskG, maskR, maskD, maskT, mask;
	vector< vector<Point> > contours;
	threshold(BGRbands[0],maskB,90,255,THRESH_BINARY);
	threshold(BGRbands[1],maskG,90,255,THRESH_BINARY);
	threshold(BGRbands[2],maskR,90,255,THRESH_BINARY);
	absdiff(BGRbands[2],BGRbands[0],maskD);
	threshold(maskD,maskD,25,255,THRESH_BINARY);
	bitwise_not(maskD,maskD);
	bitwise_and(maskR,maskD,maskD);
	bitwise_and(maskB,maskR,maskT);
	bitwise_or(maskD,maskT,maskT);
	findContours(maskT, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
	vector<double> areas = computeArea(contours);
	for(int j = areas.size()-1; j>=0; j--){
		if(areas.at(j)>10000 || areas.at(j)<400 )
			contours.erase(contours.begin()+j);
	}
	Mat out = Mat::zeros(Size(img.cols,img.rows), CV_8U);
	for (int idx = 0; idx < contours.size(); idx++)
		drawContours(out, contours, idx, Scalar(255,255,255), CV_FILLED, 8);
	return out;
 }

void detect2(Mat img, vector<Mat>& regionsOfInterest,Blob& blob){
	/*************INIZIALIZZAZIONI**********/
	Mat gray; 
	Mat out = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat masked = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat morph = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat bwmorph = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat cont = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat maskHSV = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat whiteMaskMasked = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat whiteMaskOrig = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat Bands[3];
	Mat noBackMask = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat kernelEr = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
	Mat thMasked; Mat thOrig; Mat bwOrig; Mat bwNoBackMask;
	cout<<kernelEr<<endl;
	Mat kernelOp = getStructuringElement(MORPH_ELLIPSE,Size(13,13));
	vector<Mat> BGRbands;  split(img,BGRbands);
	vector< vector<Point> > contours;
	Blob b; b.originalImage=img;
	/***************************************/
	/*cvtColor(img,gray,CV_BGR2GRAY);
	gray = (gray!=0);
	imshow("gray",gray);*/
	/*Rimozione Ombre e Background*/
//	masked = applyMaskBandByBand(maskHSV,BGRbands); split(masked,BGRbands);
	
	/*Rimozione sfondo e sogliatura per videnziare esclusivamente ciò che è bianco*/
	noBackMask = backgroundRemoval(img);
	masked = applyMaskBandByBand(noBackMask,BGRbands);
	whiteMaskOrig = computeWhiteMaskLight(img);
	whiteMaskOrig = whiteMaskOrig + computeWhiteMaskShadow(img);

	whiteMaskMasked = computeWhiteMaskLight(masked);
	whiteMaskMasked = whiteMaskMasked + computeWhiteMaskShadow(masked);

	thOrig = whiteMaskOrig-whiteMaskMasked; //Stiamo estraendo tutto il bianco dall'immagine originale che è stato perso nella mascheratura
	vector<Mat> multiThOrig; multiThOrig.push_back(thOrig); multiThOrig.push_back(thOrig); multiThOrig.push_back(thOrig);
	Mat origReplicTh =  Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	merge(multiThOrig,origReplicTh);

	/*Ora che abbiamo evidenziato le cose bianche, possiamo sommare all'immagine mascherata originale in modo da riottenere le mele*/
	masked = masked + origReplicTh;
#if DO_MORPH
	/*Operazioni morfologiche per poter riempire i buchi e rimuovere i bordi frastagliati*/
	dilate(masked,morph,kernelEr);
	erode(morph,morph,kernelEr);
	
	erode(morph,morph,kernelOp);
	dilate(morph,morph,kernelOp);
#else
	morph = masked;
#endif

	/*Ricerca componenti connesse e rimozione in base all'area*/
	cvtColor(morph,bwmorph,CV_BGR2GRAY);
	findContours(bwmorph, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
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
		Scalar color(255);
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
		if(tmpRect.x>0 && tmpRect.y>0 && tmpRect.x+tmpRect.width < morph.cols && tmpRect.y+tmpRect.height < morph.rows){ //Se il nuovo rettangolo allargato
																													// NON esce fuori, accettalo
			regionsOfInterest.push_back(img(tmpRect));
			b.cuttedImages.push_back(img(tmpRect));
			b.blobsImage.push_back(cont(tmpRect));
			b.rectangles.push_back(tmpRect);
			toPrint = tmpRect;
		}
		else{
			toPrint = boundRect[idx];
			regionsOfInterest.push_back(img(boundRect[idx]));
			b.cuttedImages.push_back(img(boundRect[idx]));
			b.rectangles.push_back(boundRect[idx]);
			b.blobsImage.push_back(cont(boundRect[idx]));
		}
		Point centroid = computeCentroid(contours[idx]);
		b.centroid.push_back(centroid);
		b.area.push_back(contourArea(contours[idx]));
		b.distance.push_back(HEIGH - centroid.y);
		
		/*rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
		circle( cont, center[idx], (int)radius[idx], color, 2, 8, 0 );*/

	}
	blob = b;
	//out = out+cont;
	bitwise_xor(out,cont,out);
	
	/*namedWindow("out",WINDOW_NORMAL);
	imshow("out",out);
	waitKey(0);*/
}

void detect3(Mat img, vector<Mat>& regionsOfInterest,Blob& blob){
	/*************INIZIALIZZAZIONI**********/
	Mat gray; 
	Mat out = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat masked = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat morph = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat bwmorph = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat cont = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat maskHSV = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat noBackMask = Mat::zeros(Size(WIDTH,HEIGH), CV_8U);
	Mat kernelEr = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
	Mat thMasked; Mat thOrig; Mat bwOrig; Mat bwNoBackMask;
	cout<<kernelEr<<endl;
	Mat kernelOp = getStructuringElement(MORPH_ELLIPSE,Size(13,13));
	vector<Mat> BGRbands;  split(img,BGRbands);
	vector< vector<Point> > contours;
	maskHSV		  = detectShadows(img);
	noBackMask    = backgroundRemoval(img);
	Blob b; b.originalImage=img;
	/***************************************/
	/*cvtColor(img,gray,CV_BGR2GRAY);
	gray = (gray!=0);
	imshow("gray",gray);*/
	/*Rimozione Ombre e Background*/
//	masked = applyMaskBandByBand(maskHSV,BGRbands); split(masked,BGRbands);
	/// Reduce noise with a kernel 3x3
	Mat src_gray,detected_edges;
	cvtColor( img, src_gray, CV_BGR2GRAY );
	int edgeThresh = 1;
	int lowThreshold = 10;
	int max_lowThreshold = 60;
	int ratio = 3;
	int kernel_size = 3;
	char* window_name = "Edge Map";
	/// Canny detector
	Canny( src_gray, detected_edges, lowThreshold, max_lowThreshold, kernel_size, true );
	/*Operazioni morfologiche per poter riempire i buchi e rimuovere i bordi frastagliati*/
	findContours(detected_edges, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
	dilate(detected_edges,detected_edges,kernelEr);
	erode(detected_edges,detected_edges,kernelEr);

	imshow("or",img);
	waitKey(0);
	imshow("det",detected_edges);
	waitKey(0);
	vector<double> areas = computeArea(contours);

	/*Calcolo Bounding Rectangle a partire dall'immagine con componenti connesse di interesse*/
	 vector<Rect> boundRect( contours.size() );
	 vector<vector<Point> > contours_poly( contours.size() );
	 vector<Point2f>center( contours.size() ); 
	 vector<float>radius( contours.size() );
	 /*Costruzione immagine finale ed estrazione regioni di interesse*/
	for (int idx = 0; idx < contours.size(); idx++){
		Scalar color(255);
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
		if(tmpRect.x>0 && tmpRect.y>0 && tmpRect.x+tmpRect.width < morph.cols && tmpRect.y+tmpRect.height < morph.rows){ //Se il nuovo rettangolo allargato
																													// NON esce fuori, accettalo
			regionsOfInterest.push_back(img(tmpRect));
			b.cuttedImages.push_back(img(tmpRect));
			b.blobsImage.push_back(cont(tmpRect));
			b.rectangles.push_back(tmpRect);
			toPrint = tmpRect;
		}
		else{
			toPrint = boundRect[idx];
			regionsOfInterest.push_back(img(boundRect[idx]));
			b.cuttedImages.push_back(img(boundRect[idx]));
			b.rectangles.push_back(boundRect[idx]);
			b.blobsImage.push_back(cont(boundRect[idx]));
		}
		Point centroid = computeCentroid(contours[idx]);
		b.centroid.push_back(centroid);
		b.area.push_back(contourArea(contours[idx]));
		b.distance.push_back(HEIGH - centroid.y);
		
		/*rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
		circle( cont, center[idx], (int)radius[idx], color, 2, 8, 0 );*/

	}
	blob = b;
	//out = out+cont;
	bitwise_xor(out,cont,out);
	
	/*namedWindow("out",WINDOW_NORMAL);
	imshow("out",out);
	waitKey(0);*/
}


//void detect(Mat img, vector<Mat>& regionsOfInterest){
//	/*************INIZIALIZZAZIONI**********/
//	Mat gray, hist;
//
//	Mat out = Mat::zeros(Size(1280,960), CV_8U);
//	Mat bin = Mat::zeros(Size(1280,960), CV_8U);
//	Mat morph = Mat::zeros(Size(1280,960), CV_8U);
//	Mat cont = Mat::zeros(Size(1280,960), CV_8U);
//	Mat maskHSV = Mat::zeros(Size(1280,960), CV_8U);
//	Mat rationedImage = Mat::zeros(Size(1280,960), CV_8U);
//	Mat noBackMask;
//	Mat tmp; vector<Mat> BGRbands;  split(img,BGRbands);
//	Mat kernelEr = getStructuringElement(MORPH_ELLIPSE,Size(2,2));
//	Mat kernelOp = getStructuringElement(MORPH_ELLIPSE,Size(5,5));
//	int histSize = 256; float range[] = { 0, 256 }; const float* histRange = { range };
//	vector< vector<Point> > contours;
//	vector<int> indexes;
//	
//	/***************************************/
//	//rationedImage = computeRationedImage(BGRbands);
//	maskHSV		  = detectShadows(img);
//	noBackMask    = backgroundRemoval(img);
//	/*
//	tmp = applyMaskBandByBand(maskHSV,BGRbands);
//	split(tmp,BGRbands);*/
//	//tmp = applyMaskBandByBand(noBackMask,BGRbands);
//	
//	/*Abbassamento risoluzione radiometrica per rimozione dettagli*/
//	tmp = img/255;
//	tmp = tmp*240+15; //+15 per mappare il nero nel bin 15 in modo da non perdere eventuali oggettineri
//	imshow("tmp",tmp);
//	cvtColor(tmp,gray,CV_BGR2GRAY);
//	//gray = gray + rationedImage;
//	gray = (gray!=0);
//	imshow("gray",gray);
//	/*Calcolo histogramma per identificazione valori di pixel significativi ( != 0 )*/
//	calcHist(&gray,1,0,Mat(),hist,1,&histSize,&histRange);
//
//	/*Calcolo gli indici corrispondenti ai bin dove c'è almeno un valore.*/
//	
//	for(int r=0; r<hist.rows;r++){ /*Escludo bin 0 -> background****** da non escludere se dobbiamo fare detection di cose nere*/
//		for(int c=0; c<hist.cols;c++)
//			if(hist.at<int>(r,c)!=0)
//				indexes.push_back(r);
//	}
//	/*Estrazione componenti connesse di interesse*/
//	
//	for(int i=0;i<indexes.size();i++){
//	
//	    /*Calcolo immagine binaria del singolo bin */
//		bin = gray == indexes[i];
//		/*Rimozione Ombre*/
//		
//		bitwise_and(bin, maskHSV ,bin);
//		bitwise_and(bin, noBackMask ,bin);
//	
//		/*Operazioni Morfologiche, kernel circolare*/
//		morphologyEx(bin, morph, MORPH_ERODE, kernelEr, Point(-1, -1));
//		morphologyEx(morph, morph, MORPH_OPEN, kernelOp, Point(-1, -1));
//
//		/*Ricerca componenti connesse come meno di un certo numero di pixel*/
//		
//		findContours(morph, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
//
//		vector<double> areas = computeArea(contours);
//		for(int j = areas.size()-1; j>=0; j--){
//			if(areas.at(j)>MAX_AREA || areas.at(j)<MIN_AREA )
//				contours.erase(contours.begin()+j);
//		}
//
//		/*Calcolo Bounding Rectangle a partire dall'immagine con componenti connesse di interesse*/
//		 vector<Rect> boundRect( contours.size() );
//		 vector<vector<Point> > contours_poly( contours.size() );
//		 vector<Point2f>center( contours.size() ); 
//		 vector<float>radius( contours.size() );
//
//		/*Costruzione immagine finale ed estrazione regioni di interesse*/
//		for (int idx = 0; idx < contours.size(); idx++){
//			Scalar color(indexes[i]);
//			approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
//			boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );
//			minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
//		//	Rect tmpRect(center[idx].x-boundRect[idx].width/2,center[idx].y-boundRect[idx].height/2,boundRect[idx].width,boundRect[idx].height);
//			Rect tmpRect(center[idx].x-radius[idx],center[idx].y-radius[idx],radius[idx]*2,radius[idx]*2);
//			//Rect tmpRect = boundRect[idx];
//			Rect toPrint;
//			tmpRect += Size(tmpRect.width*RECT_AUGMENT ,tmpRect.height*RECT_AUGMENT);			  // Aumenta area di RECT_ARGUMENT
//			tmpRect -= Point((tmpRect.width*RECT_AUGMENT)/2 , (tmpRect.height*RECT_AUGMENT)/2 ); // Ricentra il rettangolo
//			
//
//			drawContours(cont, contours, idx, color, CV_FILLED, 8);
//
//			if(tmpRect.x>0 && tmpRect.y>0 && tmpRect.x+tmpRect.width < tmp.cols && tmpRect.y+tmpRect.height < tmp.rows){ //Se il nuovo rettangolo allargato
//																														// NON esce fuori, accettalo
//				regionsOfInterest.push_back(morph(tmpRect));
//				toPrint = tmpRect;
//			}
//			else{
//				toPrint = boundRect[idx];
//				regionsOfInterest.push_back(morph(boundRect[idx]));
//			}
//			rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
//			circle( cont, center[idx], (int)radius[idx], color, 2, 8, 0 );
//
//		}
//		//out = out+cont;
//		bitwise_xor(out,cont,out);
//	}
//	namedWindow("out",WINDOW_NORMAL);
//	imshow("out",out);
//}

