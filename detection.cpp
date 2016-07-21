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
double getThreshVal_Otsu_8u( const cv::Mat& _src )
{
	cv::Size size = _src.size();
	if ( _src.isContinuous() )
	{
		size.width *= size.height;
		size.height = 1;
	}
	const int N = 256;
	int i, j, h[N] = {0};
	for ( i = 0; i < size.height; i++ )
	{
		const uchar* src = _src.data + _src.step*i;
		for ( j = 0; j <= size.width - 4; j += 4 )
		{
			int v0 = src[j], v1 = src[j+1];
			h[v0]++; h[v1]++;
			v0 = src[j+2]; v1 = src[j+3];
			h[v0]++; h[v1]++;
		}
		for ( ; j < size.width; j++ )
			h[src[j]]++;
	}

	double mu = 0, scale = 1./(size.width*size.height);
	for ( i = 0; i < N; i++ )
		mu += i*h[i];

	mu *= scale;
	double mu1 = 0, q1 = 0;
	double max_sigma = 0, max_val = 0;

	for ( i = 0; i < N; i++ )
	{
		double p_i, q2, mu2, sigma;

		p_i = h[i]*scale;
		mu1 *= q1;
		q1 += p_i;
		q2 = 1. - q1;

		if ( std::min(q1,q2) < FLT_EPSILON || std::max(q1,q2) > 1. - FLT_EPSILON )
			continue;

		mu1 = (mu1 + i*p_i)/q1;
		mu2 = (mu - q1*mu1)/q2;
		sigma = q1*q2*(mu1 - mu2)*(mu1 - mu2);
		if ( sigma > max_sigma )
		{
			max_sigma = sigma;
			max_val = i;
		}
	}

	return max_val;
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
/*
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
*/
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
		if(areas.at(j)>MAX_AREA || areas.at(j)<MIN_AREA ) // 10000 400
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
		if(areas.at(j)>MAX_AREA || areas.at(j)<MIN_AREA )
			contours.erase(contours.begin()+j);
	}
	Mat out = Mat::zeros(Size(img.cols,img.rows), CV_8U);
	for (int idx = 0; idx < contours.size(); idx++)
		drawContours(out, contours, idx, Scalar(255,255,255), CV_FILLED, 8);
	return out;
}
CBlobResult computeWhiteMaskOtsu(Mat& imgRGBin, Mat& imgHSVIn, CBlobResult& blobs, int limitRGB, int limitHSV, double RGBratio, double HSVratio, int bmin, int bmax, int i){
	Mat BGRbands[3];  
	split(imgRGBin,BGRbands);
	Mat imgHSV;
	cvtColor(imgHSVIn,imgHSV,CV_BGR2HSV);
	Mat HSVbands[3];  
	split(imgHSV,HSVbands);
	Mat maskHSV, maskRGB, maskT;

	int otsuTRGB = getThreshVal_Otsu_8u(BGRbands[2]);
	do{
		threshold(BGRbands[2],maskRGB,otsuTRGB,255,THRESH_BINARY);
		otsuTRGB++;
	}while(countNonZero(maskRGB)>(RGBratio*limitRGB) & otsuTRGB<=255);
	int otsuTHSV = getThreshVal_Otsu_8u(HSVbands[1]);
	do{	
		threshold(HSVbands[1],maskHSV,otsuTHSV,255,THRESH_BINARY_INV);
		otsuTHSV--;
	}while(countNonZero(maskHSV)>(HSVratio*limitHSV) & otsuTHSV>=0); // 0.1
	bitwise_or(maskHSV,maskRGB,maskT);
	int blobSizeBefore = blobs.GetNumBlobs();
	blobs = blobs + CBlobResult( maskT ,Mat(),8);
	blobs.Filter( blobs, B_EXCLUDE, CBlobGetLength(), B_GREATER, bmax );
	blobs.Filter( blobs, B_EXCLUDE, CBlobGetLength(), B_LESS, bmin );
	int blobSizeAfter = blobs.GetNumBlobs();
	Mat newMask(maskT.size(),maskT.type());
	newMask.setTo(0);
	for(;i<blobs.GetNumBlobs();i++){
		double area = blobs.GetBlob(i)->Area();
		if(area < 11000 && area > 1000)
			blobs.GetBlob(i)->FillBlob(newMask,CV_RGB(255,255,255),0,0,true);
	} 
	if(countNonZero(maskRGB)>400 && countNonZero(maskHSV)>400 && blobSizeBefore!=blobSizeAfter){
		vector<Mat> BGRbands;  split(imgRGBin,BGRbands);
		Mat maskedRGB = applyMaskBandByBand(newMask,BGRbands);
		bitwise_not(newMask,newMask);
		split(imgHSVIn,BGRbands);
		Mat maskedHSV = applyMaskBandByBand(newMask,BGRbands);
		blobs = computeWhiteMaskOtsu(maskedRGB, maskedHSV, blobs, countNonZero(maskRGB),countNonZero(maskHSV),RGBratio, HSVratio, bmin, bmax, i-1);
	}		
	return blobs;
}
void rgb2cmyk(cv::Mat& img, std::vector<cv::Mat>& cmyk) {
	// Allocate cmyk to store 4 componets
	for (int i = 0; i < 4; i++) {
		cmyk.push_back(cv::Mat(img.size(), CV_8UC1));
	}

	// Get rgb
	std::vector<cv::Mat> rgb;
	cv::split(img, rgb);

	// rgb to cmyk
	for (int i = 0; i < img.rows; i++) {
		for (int j = 0; j < img.cols; j++) {
			float r = (int)rgb[2].at<uchar>(i, j) / 255.;
			float g = (int)rgb[1].at<uchar>(i, j) / 255.;
			float b = (int)rgb[0].at<uchar>(i, j) / 255.;
			float k = std::min(std::min(1- r, 1- g), 1- b);         

			cmyk[0].at<uchar>(i, j) = (1 - r - k) / (1 - k) * 255.;
			cmyk[1].at<uchar>(i, j) = (1 - g - k) / (1 - k) * 255.;
			cmyk[2].at<uchar>(i, j) = (1 - b - k) / (1 - k) * 255.;
			cmyk[3].at<uchar>(i, j) = k * 255.;
		}
	}
}
Mat computeInterest(Mat& img){
	std::vector<cv::Mat> cmyk;
	rgb2cmyk(img, cmyk);
	int otsuT = getThreshVal_Otsu_8u(cmyk.at(2));
	Mat out;
	threshold(cmyk.at(2),out,otsuT,255,THRESH_BINARY);
	return img;
}
void detect2(Mat img, vector<Mat>& regionsOfInterest,vector<Blob>& blobs){
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
	Mat kernelOp = getStructuringElement(MORPH_ELLIPSE,Size(13,13));
	vector<Mat> BGRbands;  split(img,BGRbands);
	vector< vector<Point> > contours;
	/***************************************/
	/*cvtColor(img,gray,CV_BGR2GRAY);
	gray = (gray!=0);
	imshow("gray",gray);*/
	/*Rimozione Ombre e Background*/
	//	masked = applyMaskBandByBand(maskHSV,BGRbands); split(masked,BGRbands);

	/*Rimozione sfondo e sogliatura per videnziare esclusivamente ciò che è bianco*/
	noBackMask = backgroundRemoval(img);
	masked = applyMaskBandByBand(noBackMask,BGRbands);
	/*
	whiteMaskOrig = computeWhiteMaskLight(img);
	whiteMaskOrig = whiteMaskOrig + computeWhiteMaskShadow(img);

	whiteMaskMasked = computeWhiteMaskLight(masked);
	whiteMaskMasked = whiteMaskMasked + computeWhiteMaskShadow(masked);
	*/
	computeInterest(img);
	CBlobResult blobsRs;
	blobsRs = computeWhiteMaskOtsu(img, img, blobsRs, img.rows*img.cols, img.rows*img.cols, 0.8, 0.8, 30, 200, 0);

	whiteMaskOrig.setTo(0);
	for(int i=0;i<blobsRs.GetNumBlobs();i++){
		double area = blobsRs.GetBlob(i)->Area();
		if(area < 10000 && area > 400)
			blobsRs.GetBlob(i)->FillBlob(whiteMaskOrig,CV_RGB(255,255,255),0,0,true);
	}

	threshold(masked,whiteMaskMasked,0,255,THRESH_BINARY);
	cvtColor(whiteMaskMasked,whiteMaskMasked,CV_BGR2GRAY);
	cout << whiteMaskMasked.type() << " " << whiteMaskOrig.type() << endl;
	bitwise_or(whiteMaskMasked,whiteMaskOrig,thOrig);
	masked = applyMaskBandByBand(thOrig,BGRbands);
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
		Blob b; b.originalImage = img;
		Scalar color(255);
		approxPolyDP( Mat(contours[idx]), contours_poly[idx], 3, true );
		boundRect[idx] = boundingRect( Mat(contours_poly[idx]) );

		minEnclosingCircle( (Mat)contours_poly[idx], center[idx], radius[idx] );
		//	Rect tmpRect(center[idx].x-boundRect[idx].width/2,center[idx].y-boundRect[idx].height/2,boundRect[idx].width,boundRect[idx].height);
		Rect tmpRect(center[idx].x-radius[idx],center[idx].y-radius[idx],radius[idx]*2,radius[idx]*2);
		//Rect tmpRect = boundRect[idx];
		Rect toPrint = resizeRectangle(tmpRect);
		drawContours(cont, contours, idx, color, CV_FILLED, 8);
		rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
		circle( cont, center[idx], (int)radius[idx], color, 2, 8, 0 );

		regionsOfInterest.push_back(img(toPrint));
		b.cuttedWithBack = img(toPrint);
		b.cuttedImages = img(toPrint);
		b.blobsImage = cont(toPrint);
		b.rectangles = toPrint;
		b.resizedRect = refitToBorders(img(toPrint));
		Point centroid = computeCentroid(contours[idx]);
		b.centroid = centroid;
		b.area = contourArea(contours[idx]); // Va bene calcolare queste due features senza fittare i bordi in quanto usiamo direttamente
		// l'output della findContours. 
		b.distance = HEIGH - centroid.y;


		/*rectangle( cont, toPrint.tl(), toPrint.br(), color, 2, 8, 0 );
		circle( cont, center[idx], (int)radius[idx], color, 2, 8, 0 );*/
		if(!(b.cuttedImages(b.resizedRect).rows*b.cuttedImages(b.resizedRect).cols)<400)
			blobs.push_back(b);
		else{
			cout<<"VIAAA"<<endl;
			imshow("gettato",b.cuttedImages(b.resizedRect)) ;
			waitKey(0);
			cvDestroyAllWindows();

		}

		}
		bitwise_xor(out,cont,out);
		/*imshow("img",img);
		imshow("out",out);
		waitKey(0);
		cvDestroyAllWindows();*/
	}

	Rect resizeRectangle(Rect r){
		Rect tmpRect = r;

		tmpRect += Size(r.width*RECT_AUGMENT ,r.height*RECT_AUGMENT);			  // Aumenta area di RECT_ARGUMENT
		tmpRect -= Point((r.width*RECT_AUGMENT)/2 , (r.height*RECT_AUGMENT)/2 ); // Ricentra il rettangolo
		do{
			if(tmpRect.br().x>WIDTH ||  tmpRect.br().y>HEIGH ){//|| tmpRect.tl().y<0 |||tmpRect.tl().x<0  ){ // Se il rect esce fuori dall'immagine
				Point p(tmpRect.br().x-1, tmpRect.br().y-1);
				tmpRect = Rect(tmpRect.tl(),p);
			}
			if (tmpRect.tl().y<0 ||tmpRect.tl().x<0  ){
				Point p(tmpRect.tl().x+1, tmpRect.tl().y+1);
				tmpRect = Rect(p,tmpRect.br());
			}
		}while( tmpRect.br().x>WIDTH ||  tmpRect.br().y>HEIGH || tmpRect.tl().y<0 ||tmpRect.tl().x<0  );
		if(tmpRect.area()< 500)
			cout<<"ALARM"<<endl;
		return tmpRect;
	}

	/* Computes a bounding rectangle for the object according to its borders */
	Rect refitToBorders(Mat region) {
		Mat img;
		cvtColor(region, img, CV_BGR2HSV);
		medianBlur(img, img, 11);
		Laplacian(img, img, -1, 3);
		Canny(img, img, 100, 200);

		int minX = img.cols, maxX = 0, minY = img.rows, maxY = 0;
		for (size_t x = 0; x < img.cols; x++) {
			for (size_t y = 0; y < img.rows; y++) {
				if (!img.at<uchar>(y, x))
					continue;
				if (x < minX)
					minX = x;
				if (x > maxX)
					maxX = x;
				if (y < minY)
					minY = y;
				if (y > maxY)
					maxY = y;
			}
		}
		return Rect(Point(minX, minY), Point(maxX, maxY));
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

