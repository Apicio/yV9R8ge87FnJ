// SLIC.cpp: implementation of the SLIC class.
//
// Copyright (C) Radhakrishna Achanta 2012
// All rights reserved
// Email: firstname.lastname@epfl.ch
//////////////////////////////////////////////////////////////////////
#include <cfloat>
#include <cmath>
#include <iostream>
#include <fstream>
#include "SLIC.h"


//////////////////////////////////////////////////////////////////////
// Construction/Destruction
//////////////////////////////////////////////////////////////////////

SLIC::SLIC()
{
	m_lvec = NULL;
	m_avec = NULL;
	m_bvec = NULL;

	m_lvecvec = NULL;
	m_avecvec = NULL;
	m_bvecvec = NULL;
}

SLIC::~SLIC()
{
	if(m_lvec) delete [] m_lvec;
	if(m_avec) delete [] m_avec;
	if(m_bvec) delete [] m_bvec;


	if(m_lvecvec)
	{
		for( int d = 0; d < m_depth; d++ ) delete [] m_lvecvec[d];
		delete [] m_lvecvec;
	}
	if(m_avecvec)
	{
		for( int d = 0; d < m_depth; d++ ) delete [] m_avecvec[d];
		delete [] m_avecvec;
	}
	if(m_bvecvec)
	{
		for( int d = 0; d < m_depth; d++ ) delete [] m_bvecvec[d];
		delete [] m_bvecvec;
	}
}

//==============================================================================
///	RGB2XYZ
///
/// sRGB (D65 illuninant assumption) to XYZ conversion
//==============================================================================
void SLIC::RGB2XYZ(
	const int&		sR,
	const int&		sG,
	const int&		sB,
	double&			X,
	double&			Y,
	double&			Z)
{
	double R = sR/255.0;
	double G = sG/255.0;
	double B = sB/255.0;

	double r, g, b;

	if(R <= 0.04045)	r = R/12.92;
	else				r = pow((R+0.055)/1.055,2.4);
	if(G <= 0.04045)	g = G/12.92;
	else				g = pow((G+0.055)/1.055,2.4);
	if(B <= 0.04045)	b = B/12.92;
	else				b = pow((B+0.055)/1.055,2.4);

	X = r*0.4124564 + g*0.3575761 + b*0.1804375;
	Y = r*0.2126729 + g*0.7151522 + b*0.0721750;
	Z = r*0.0193339 + g*0.1191920 + b*0.9503041;
}

//===========================================================================
///	RGB2LAB
//===========================================================================
void SLIC::RGB2LAB(const int& sR, const int& sG, const int& sB, double& lval, double& aval, double& bval)
{
	//------------------------
	// sRGB to XYZ conversion
	//------------------------
	double X, Y, Z;
	RGB2XYZ(sR, sG, sB, X, Y, Z);

	//------------------------
	// XYZ to LAB conversion
	//------------------------
	double epsilon = 0.008856;	//actual CIE standard
	double kappa   = 903.3;		//actual CIE standard

	double Xr = 0.950456;	//reference white
	double Yr = 1.0;		//reference white
	double Zr = 1.088754;	//reference white

	double xr = X/Xr;
	double yr = Y/Yr;
	double zr = Z/Zr;

	double fx, fy, fz;
	if(xr > epsilon)	fx = pow(xr, 1.0/3.0);
	else				fx = (kappa*xr + 16.0)/116.0;
	if(yr > epsilon)	fy = pow(yr, 1.0/3.0);
	else				fy = (kappa*yr + 16.0)/116.0;
	if(zr > epsilon)	fz = pow(zr, 1.0/3.0);
	else				fz = (kappa*zr + 16.0)/116.0;

	lval = 116.0*fy-16.0;
	aval = 500.0*(fx-fy);
	bval = 200.0*(fy-fz);
}

//===========================================================================
///	DoRGBtoLABConversion
///
///	For whole image: overlaoded floating point version
//===========================================================================
void SLIC::DoRGBtoLABConversion(
	const unsigned int*&		ubuff,
	double*&					lvec,
	double*&					avec,
	double*&					bvec)
{
	int sz = m_width*m_height;
	lvec = new double[sz];
	avec = new double[sz];
	bvec = new double[sz];

	for( int j = 0; j < sz; j++ )
	{
		int r = (ubuff[j] >> 16) & 0xFF;
		int g = (ubuff[j] >>  8) & 0xFF;
		int b = (ubuff[j]      ) & 0xFF;

		RGB2LAB( r, g, b, lvec[j], avec[j], bvec[j] );
	}
}

//===========================================================================
///	DoRGBtoLABConversion
///
/// For whole volume
//===========================================================================
void SLIC::DoRGBtoLABConversion(
	unsigned int**&		ubuff,
	double**&					lvec,
	double**&					avec,
	double**&					bvec)
{
	int sz = m_width*m_height;
	for( int d = 0; d < m_depth; d++ )
	{
		for( int j = 0; j < sz; j++ )
		{
			int r = (ubuff[d][j] >> 16) & 0xFF;
			int g = (ubuff[d][j] >>  8) & 0xFF;
			int b = (ubuff[d][j]      ) & 0xFF;

			RGB2LAB( r, g, b, lvec[d][j], avec[d][j], bvec[d][j] );
		}
	}
}

//=================================================================================
/// DrawContoursAroundSegments
///
/// Internal contour drawing option exists. One only needs to comment the if
/// statement inside the loop that looks at neighbourhood.
//=================================================================================
void SLIC::DrawContoursAroundSegments(
	unsigned int*&			ubuff,
	int*&					labels,
	const int&				width,
	const int&				height,
	const unsigned int&				color )
{
	const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

/*	int sz = width*height;

	vector<bool> istaken(sz, false);

	int mainindex(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			int np(0);
			for( int i = 0; i < 8; i++ )
			{
				int x = k + dx8[i];
				int y = j + dy8[i];

				if( (x >= 0 && x < width) && (y >= 0 && y < height) )
				{
					int index = y*width + x;

					if( false == istaken[index] )//comment this to obtain internal contours
					{
						if( labels[mainindex] != labels[index] ) np++;
					}
				}
			}
			if( np > 1 )//change to 2 or 3 for thinner lines
			{
				ubuff[mainindex] = color;
				istaken[mainindex] = true;
			}
			mainindex++;
		}
	}*/


	int sz = width*height;
	vector<bool> istaken(sz, false);
	vector<int> contourx(sz);vector<int> contoury(sz);
	int mainindex(0);int cind(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			int np(0);
			for( int i = 0; i < 8; i++ )
			{
				int x = k + dx8[i];
				int y = j + dy8[i];

				if( (x >= 0 && x < width) && (y >= 0 && y < height) )
				{
					int index = y*width + x;

					//if( false == istaken[index] )//comment this to obtain internal contours
					{
						if( labels[mainindex] != labels[index] ) np++;
					}
				}
			}
			if( np > 1 )
			{
				contourx[cind] = k;
				contoury[cind] = j;
				istaken[mainindex] = true;
				//img[mainindex] = color;
				cind++;
			}
			mainindex++;
		}
	}

	int numboundpix = cind;//int(contourx.size());
	for( int j = 0; j < numboundpix; j++ )
	{
		int ii = contoury[j]*width + contourx[j];
		ubuff[ii] = 0xffffff;

		for( int n = 0; n < 8; n++ )
		{
			int x = contourx[j] + dx8[n];
			int y = contoury[j] + dy8[n];
			if( (x >= 0 && x < width) && (y >= 0 && y < height) )
			{
				int ind = y*width + x;
				if(!istaken[ind]) ubuff[ind] = 0;
			}
		}
	}
}

//=================================================================================
/// DrawContoursAroundSegments
///
/// Internal contour drawing option exists. One only needs to comment the if
/// statement inside the loop that looks at neighbourhood.
//=================================================================================
void SLIC::DrawContoursAroundSegments(
	unsigned int*			ubuff,
	const int*				labels,
	const int&				width,
	const int&				height,
	const cv::Scalar&		color )
{
	const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

	int sz = width*height;

	vector<bool> istaken(sz, false);

	int mainindex(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			int np(0);
			for( int i = 0; i < 8; i++ )
			{
				int x = k + dx8[i];
				int y = j + dy8[i];

				if( (x >= 0 && x < width) && (y >= 0 && y < height) )
				{
					int index = y*width + x;

					if( false == istaken[index] )//comment this to obtain internal contours
					{
						if( labels[mainindex] != labels[index] ) np++;
					}
				}
			}
			if( np > 1 )//change to 2 or 3 for thinner lines
			{
				ubuff[mainindex] = 0;
				ubuff[mainindex] |= (int)color.val[2] << 16; // r
				ubuff[mainindex] |= (int)color.val[1] << 8; // g
				ubuff[mainindex] |= (int)color.val[0];
				//ubuff[mainindex] |= 255 << 16; // r
				//ubuff[mainindex] |= 0 << 8; // g
				//ubuff[mainindex] |= 0;
				istaken[mainindex] = true;
			}
			mainindex++;
		}
	}
}

void SLIC::DrawContoursAroundSegments(
	unsigned char*			ubuff,
	const int*				labels,
	const int&				width,
	const int&				height,
	const cv::Scalar&		color )
{
	const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

	int sz = width*height;

	vector<bool> istaken(sz, false);

	int mainindex(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			int np(0);
			for( int i = 0; i < 8; i++ )
			{
				int x = k + dx8[i];
				int y = j + dy8[i];

				if( (x >= 0 && x < width) && (y >= 0 && y < height) )
				{
					int index = y*width + x;

					if( false == istaken[index] )//comment this to obtain internal contours
					{
						if( labels[mainindex] != labels[index] ) np++;
					}
				}
			}
			if( np > 1 )//change to 2 or 3 for thinner lines
			{
				ubuff[mainindex] = (uchar)color.val[0];
				istaken[mainindex] = true;
			}
			mainindex++;
		}
	}
}

//=================================================================================
/// DrawContoursAroundSegmentsTwoColors
///
/// Internal contour drawing option exists. One only needs to comment the if
/// statement inside the loop that looks at neighbourhood.
//=================================================================================
void SLIC::DrawContoursAroundSegmentsTwoColors(
	unsigned int*			img,
	const int*				labels,
	const int&				width,
	const int&				height)
{
	const int dx[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

	int sz = width*height;
	
	vector<bool> istaken(sz, false);
	

	vector<int> contourx(sz);
	vector<int> contoury(sz);
	int mainindex(0);
	int cind(0);
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			int np(0);
			for( int i = 0; i < 8; i++ )
			{
				int x = k + dx[i];
				int y = j + dy[i];

				if( (x >= 0 && x < width) && (y >= 0 && y < height) )
				{
					int index = y*width + x;

					//if( false == istaken[index] )//comment this to obtain internal contours
					{
						if( labels[mainindex] != labels[index] )np++;
					}
				}
			}
			if( np > 1 )
			{
				contourx[cind] = k;
				contoury[cind] = j;
				istaken[mainindex] = true;
				//img[mainindex] = color;
				cind++;
			}
			mainindex++;
		}
	}

	int numboundpix = cind;//int(contourx.size());

	for( int j = 0; j < numboundpix; j++ )
	{
		int ii = contoury[j]*width + contourx[j];
		img[ii] = 0xffffff;
		//----------------------------------
		// Uncomment this for thicker lines
		//----------------------------------
		for( int n = 0; n < 8; n++ )
		{
			int x = contourx[j] + dx[n];
			int y = contoury[j] + dy[n];
			if( (x >= 0 && x < width) && (y >= 0 && y < height) )
			{
				int ind = y*width + x;
				if(!istaken[ind]) img[ind] = 0;
			}
		}
	}
}

void SLIC::mergeSuperPixel(SuperPixel & in_out, SuperPixel & toSum){
	/* Merge Histograms */
	in_out.hist_orig = in_out.hist_orig + toSum.hist_base;
	normalize(in_out.hist_orig, in_out.hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	/* Update Label */

	/* Update Nearests */
	vector<int> nearests;
	for(int i = 0; i<in_out.nearests.size(); i++)
		if(in_out.nearests.at(i) != toSum.label)
			nearests.push_back(in_out.nearests.at(i));

	for(int i = 0; i<toSum.nearests.size(); i++)
		if(toSum.nearests.at(i) != in_out.label)
			nearests.push_back(toSum.nearests.at(i));
	in_out.nearests = nearests;
	
	/* Update Pixels */
	in_out.points.insert(in_out.points.end(), toSum.points.begin(), toSum.points.end());
	toSum.points.clear();
}


void scanColor(cv::Mat& rr, int id_col, int id_row, bool**& sims, int size, vector<SuperPixel > &bagOfSuperPixel, int value, bool isRowScan){
	if(isRowScan){
		for(int i=0; i<size; i++){
			if(sims[id_row][i]){
				sims[id_row][i] = false;
				SuperPixel ss1 = bagOfSuperPixel.at(i);
				SuperPixel ss2 = bagOfSuperPixel.at(id_row);
				for(int k=0; k<ss1.points.size(); k++){
					cv::Point p = ss1.points.at(k);
					rr.at<uchar>(p.y,p.x) = value;			
				}
				for(int k=0; k<ss2.points.size(); k++){
					cv::Point p = ss2.points.at(k);
					rr.at<uchar>(p.y,p.x) = value;			
				}
				scanColor(rr, i, id_row, sims, size, bagOfSuperPixel, value, false);
			}
		}
	}else{
		for(int i=0; i<size; i++){
			if(sims[i][id_col]){
				sims[i][id_col] = false;
				SuperPixel ss1 = bagOfSuperPixel.at(i);
				SuperPixel ss2 = bagOfSuperPixel.at(id_col);
				for(int k=0; k<ss1.points.size(); k++){
					cv::Point p = ss1.points.at(k);
					rr.at<uchar>(p.y,p.x) = value;			
				}
				for(int k=0; k<ss2.points.size(); k++){
					cv::Point p = ss2.points.at(k);
					rr.at<uchar>(p.y,p.x) = value;			
				}
				scanColor(rr, id_col, i, sims, size, bagOfSuperPixel, value, true);
			}
		}
	}
}


void SLIC::GetPixelsSet(
	cv::Mat			img,
	const int*				labels,
	const int&				width,
	const int&				height,
	int numlabels, 
	vector<SuperPixel > &bagOfSuperPixel,
	bool**& in_matrix)
{	
/* Set init & ADJ Init*/
	for(int i=0; i<numlabels; i++){
		SuperPixel sp;
		bagOfSuperPixel.push_back(sp);
		bagOfSuperPixel.at(i).label = i;
	}
	int sz = width*height;
	int oldLabelIndex = 0;
	bool** matrix = new bool*[numlabels];
	for (int i = 0; i < numlabels; ++i)
		matrix[i] = new bool[numlabels];
	for(int kk = 0; kk<numlabels; kk++)
		for(int qq = 0; qq<numlabels; qq++)
			matrix[kk][qq] = false;
/* Push Pixels inside set */	
	for( int j = 0; j < height; j++ )
		for( int k = 0; k < width; k++ ){
			int index = j*width + k;
			bagOfSuperPixel.at(labels[index]).points.push_back(cv::Point(k,j));	
		}
/* Compute Adj Matrix */
	for( int j = 0; j < height; j++ )
		for( int k = 0; k < width; k++ ){
			int index = j*width + k;
			oldLabelIndex = k==0? index : oldLabelIndex;
			if(labels[oldLabelIndex] != labels[index]){
				matrix[labels[oldLabelIndex]][labels[index]] = true;
				matrix[labels[index]][labels[oldLabelIndex]] = true;
				oldLabelIndex = index;
			}
		}
	for( int k = 0; k < width; k++ )
		for( int j = 0; j < height; j++ ){
			int index = j*width + k;
			oldLabelIndex = j==0? index : oldLabelIndex;
			if(labels[oldLabelIndex] != labels[index]){
				matrix[labels[oldLabelIndex]][labels[index]] = true;
				matrix[labels[index]][labels[oldLabelIndex]] = true;
				oldLabelIndex = index;
			}
		}
	in_matrix = matrix;
	for(int i=0; i<bagOfSuperPixel.size(); i++)
		for(int kk = 0; kk<numlabels; kk++)
			if(matrix[i][kk] || matrix[kk][i])
				bagOfSuperPixel.at(i).nearests.push_back(kk);

	/* Compute Superpixel */
	for(int i=0; i<bagOfSuperPixel.size(); i++){
		vector<cv::Point > component = bagOfSuperPixel.at(i).points;
		bagOfSuperPixel.at(i).superpixel = cv::Mat::zeros(1,component.size(),CV_8UC3);
		int idx = 0;
		for(int j=0; j<component.size(); j++){
			cv::Point p = component.at(j);
				bagOfSuperPixel.at(i).superpixel.at<cv::Vec3b>(0,idx) = img.at<cv::Vec3b>(p.y,p.x);
				idx++;
	}
#if 0

	cv::Mat imgHSV, imgYUV;
	cv::Mat HSVbands[3], YUVbands[3];
	cvtColor(bagOfSuperPixel.at(i).superpixel,imgHSV,CV_BGR2HSV);
	cvtColor(bagOfSuperPixel.at(i).superpixel,imgYUV,CV_BGR2YCrCb);
	double size = bagOfSuperPixel.at(i).superpixel.rows*bagOfSuperPixel.at(i).superpixel.cols;
	split(imgHSV,HSVbands);
	split(imgYUV,YUVbands);
	cv::Mat mask1, mask2, mask3, maskTOT;
	mask1 = HSVbands[0] <= 90/2;
	mask2 = HSVbands[0] >= 270/2;
	mask3 = cv::Mat::zeros(mask2.size(),CV_8UC1);
	for(int i = 0; i<mask3.rows; i++){
		for(int j = 0; j<mask3.rows; j++){
			if(HSVbands[1].at<uchar>(i,j) < HSVbands[2].at<uchar>(i,j)*0.7)
				mask3.at<uchar>(i,j) = 255;
		}
	}
	maskTOT = mask1 + mask2 + mask3;
	int numWhite = countNonZero(maskTOT);
	double fraction = numWhite/size;
	bagOfSuperPixel.at(i).isOfInterest = fraction>0.2;
/*2*/
	double size = bagOfSuperPixel.at(i).superpixel.rows*bagOfSuperPixel.at(i).superpixel.cols;
	cv::vector<cv::Mat> channels;
    cv::split(bagOfSuperPixel.at(i).superpixel,channels);
    cv::Mat blue_channel = channels.at(0);
    cv::Mat green_channel = channels.at(1);
    cv::Mat red_channel = channels.at(2);
	cv::Mat lumMat = (0.2126 * red_channel) + (0.7152 * green_channel) + (0.0722 * blue_channel);
	//Normalised colour maps
    cv::Mat red_n = (255 / 3) * (red_channel / lumMat);
    cv::Mat green_n = (255 / 3) * (green_channel / lumMat);
    cv::Mat blue_n = (255 / 3) * (blue_channel / lumMat);
	//Calculate the end RGB components
    cv::Mat red_d = red_n - ((green_n + blue_n)/2);
    cv::Mat green_d = green_n - ((red_n + blue_n)/2);
    cv::Mat blue_d = blue_n - ((red_n + green_n)/2);
	//Calculate the difference between the red and green mat to get yellow.
    cv::Mat diffMat;
    cv::absdiff(red_n, green_n, diffMat);

    //Calulate the Y component
    cv::Mat yellow_d = (red_n / 2) + (green_n/2) - blue_n - diffMat;

    //Threshold the respective colour channels to their cut off values
    cv::threshold(red_d, red_d, 0, 255, CV_THRESH_BINARY);
    cv::threshold(green_d, green_d, 30, 255, CV_THRESH_BINARY);
    cv::threshold(blue_d, blue_d, 0, 255, CV_THRESH_BINARY);
    cv::threshold(yellow_d, yellow_d, 0, 255, CV_THRESH_BINARY);

	int numRed = countNonZero(red_d);
	double fractionRed = numRed/size;
	int numGreen = countNonZero(green_d);
	double fractionGreen = numGreen/size;
	int numYellow = countNonZero(yellow_d);
	double fractionYellow = numYellow/size;
	int numBlue = countNonZero(blue_d);
	double fractionBlue = numBlue/size;

	if(fractionRed > 0.1 || fractionYellow > 0.1)
		bagOfSuperPixel.at(i).isOfInterest = true;
	else if (fractionGreen < 0.6)
		if(fractionBlue == 0)
			bagOfSuperPixel.at(i).isOfInterest = true;
		else
			bagOfSuperPixel.at(i).isOfInterest = false;
	else
		bagOfSuperPixel.at(i).isOfInterest = false;

	}
#endif
#if 1
/* Compute Histogram */
	cv::Mat hsv_base;
	cvtColor(bagOfSuperPixel.at(i).superpixel, hsv_base, cv::COLOR_BGR2HSV );
	int h_bins = 180; int s_bins = 256;
	int histSize[] = { h_bins, s_bins };
	Mat1b HSVbands[3];
	split(hsv_base,HSVbands);
	imadjust(HSVbands[2],HSVbands[2],1, Vec2i(0, 255), Vec2i(0, 255));
	vector<Mat> multiThOrig; multiThOrig.push_back(HSVbands[0]); multiThOrig.push_back(HSVbands[1]); multiThOrig.push_back(HSVbands[2]);
	cv::merge(multiThOrig,hsv_base);
	// hue varies from 0 to 179, saturation from 0 to 255
	float h_ranges[] = { 0, 180 };
	float s_ranges[] = { 0, 256 };
	const float* ranges[] = { h_ranges, s_ranges };
	int channels[] = { 0, 1 };
	/// Histograms
	cv::MatND hist_base;
	calcHist( &hsv_base, 1, channels, cv::Mat(), hist_base, 2, histSize, ranges, true, false );
	hist_base.copyTo(bagOfSuperPixel.at(i).hist_orig);
    //normalize( hist_base, hist_base, 0, 1, cv::NORM_MINMAX, -1, cv::Mat() );
	bagOfSuperPixel.at(i).hist_base = hist_base;
	// double base_base = compareHist( hist_base, hist_base, compare_method );
}
/* Compare Histograms */
double soglia = 0.25;

bool** sims = new bool*[numlabels];
for (int i = 0; i < numlabels; ++i)
	sims[i] = new bool[numlabels];
for(int kk = 0; kk<numlabels; kk++)
	for(int qq = 0; qq<numlabels; qq++)
		sims[kk][qq] = kk==qq;

for(int i=0; i<bagOfSuperPixel.size(); i++){
		SuperPixel ss = bagOfSuperPixel.at(i);
		cv::MatND hist_curr = ss.hist_base;
		for(int j=0; j<ss.nearests.size(); j++){
			SuperPixel Near2SS = bagOfSuperPixel.at(ss.nearests.at(j));
			double val = compareHist(hist_curr, Near2SS.hist_base, CV_COMP_CORREL );
			if(val > soglia){
				sims[ss.label][Near2SS.label] = true;
				sims[Near2SS.label][ss.label] = true;
		}
	}
}
#endif
#if 1
cv::Mat rr = cv::Mat::zeros(height,width,CV_8U);
for(int i=0; i<numlabels; i++){	
	int value = rand() %175+80;
	for(int j=0; j<numlabels; j++){
		if(sims[i][j]){
			sims[i][j] = false;	
			SuperPixel ss1 = bagOfSuperPixel.at(i);
			SuperPixel ss2 = bagOfSuperPixel.at(j);
			for(int k=0; k<ss1.points.size(); k++){
				cv::Point p = ss1.points.at(k);
				rr.at<uchar>(p.y,p.x) = value;			
			}
			for(int k=0; k<ss2.points.size(); k++){
				cv::Point p = ss2.points.at(k);
				rr.at<uchar>(p.y,p.x) = value;			
			}
			int id_col = j;
			int id_row = i;
			bool isRowScan = false;
			scanColor(rr, id_col, id_row, sims, numlabels, bagOfSuperPixel, value, isRowScan);
		}
	}
}

	cv::imshow("abcdef3",rr);
	cv::waitKey(300);
#endif
	cv::imshow("abcdef",img);
	cv::waitKey(300);


/* Show Result */	

#if 0
	for(int i=0; i<set.size(); i++){
		cv::Mat rr = cv::Mat::zeros(height,width,CV_8U);
		vector<cv::Point > component = set.at(i);
		for(int j=0; j<component.size(); j++){
			cv::Point p = component.at(j);
				rr.at<uchar>(p.y,p.x) = 255;			
		}
		for(int kk = 0; kk<numlabels; kk++){
			if(matrix[i][kk] || matrix[kk][i]){
				vector<cv::Point > component = set.at(kk);
				int value = rand() %175+80;
				for(int j=0; j<component.size(); j++){
					cv::Point p = component.at(j);
					rr.at<uchar>(p.y,p.x) = value;			
				}
			}
		}
		cv::imshow("ewwiwa",rr);
		cv::waitKey(300);
	}
#endif			
}


//==============================================================================
///	DetectLabEdges
//==============================================================================
void SLIC::DetectLabEdges(
	const double*				lvec,
	const double*				avec,
	const double*				bvec,
	const int&					width,
	const int&					height,
	vector<double>&				edges)
{
	int sz = width*height;

	edges.resize(sz,0);
	for( int j = 1; j < height-1; j++ )
	{
		for( int k = 1; k < width-1; k++ )
		{
			int i = j*width+k;

			double dx = (lvec[i-1]-lvec[i+1])*(lvec[i-1]-lvec[i+1]) +
						(avec[i-1]-avec[i+1])*(avec[i-1]-avec[i+1]) +
						(bvec[i-1]-bvec[i+1])*(bvec[i-1]-bvec[i+1]);

			double dy = (lvec[i-width]-lvec[i+width])*(lvec[i-width]-lvec[i+width]) +
						(avec[i-width]-avec[i+width])*(avec[i-width]-avec[i+width]) +
						(bvec[i-width]-bvec[i+width])*(bvec[i-width]-bvec[i+width]);

			//edges[i] = fabs(dx) + fabs(dy);
			edges[i] = dx*dx + dy*dy;
		}
	}
}

//===========================================================================
///	PerturbSeeds
//===========================================================================
void SLIC::PerturbSeeds(
	vector<double>&				kseedsl,
	vector<double>&				kseedsa,
	vector<double>&				kseedsb,
	vector<double>&				kseedsx,
	vector<double>&				kseedsy,
        const vector<double>&                   edges)
{
	const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};
	
	int numseeds = kseedsl.size();

	for( int n = 0; n < numseeds; n++ )
	{
		int ox = kseedsx[n];//original x
		int oy = kseedsy[n];//original y
		int oind = oy*m_width + ox;

		int storeind = oind;
		for( int i = 0; i < 8; i++ )
		{
			int nx = ox+dx8[i];//new x
			int ny = oy+dy8[i];//new y

			if( nx >= 0 && nx < m_width && ny >= 0 && ny < m_height)
			{
				int nind = ny*m_width + nx;
				if( edges[nind] < edges[storeind])
				{
					storeind = nind;
				}
			}
		}
		if(storeind != oind)
		{
			kseedsx[n] = storeind%m_width;
			kseedsy[n] = storeind/m_width;
			kseedsl[n] = m_lvec[storeind];
			kseedsa[n] = m_avec[storeind];
			kseedsb[n] = m_bvec[storeind];
		}
	}
}


//===========================================================================
///	GetLABXYSeeds_ForGivenStepSize
///
/// The k seed values are taken as uniform spatial pixel samples.
//===========================================================================
void SLIC::GetLABXYSeeds_ForGivenStepSize(
	vector<double>&				kseedsl,
	vector<double>&				kseedsa,
	vector<double>&				kseedsb,
	vector<double>&				kseedsx,
	vector<double>&				kseedsy,
    const int&					STEP,
    const bool&					perturbseeds,
    const vector<double>&       edgemag)
{
    const bool hexgrid = false;
	int numseeds(0);
	int n(0);

	//int xstrips = m_width/STEP;
	//int ystrips = m_height/STEP;
	int xstrips = (0.5+double(m_width)/double(STEP));
	int ystrips = (0.5+double(m_height)/double(STEP));

    int xerr = m_width  - STEP*xstrips;if(xerr < 0){xstrips--;xerr = m_width - STEP*xstrips;}
    int yerr = m_height - STEP*ystrips;if(yerr < 0){ystrips--;yerr = m_height- STEP*ystrips;}

	double xerrperstrip = double(xerr)/double(xstrips);
	double yerrperstrip = double(yerr)/double(ystrips);

	int xoff = STEP/2;
	int yoff = STEP/2;
	//-------------------------
	numseeds = xstrips*ystrips;
	//-------------------------
	kseedsl.resize(numseeds);
	kseedsa.resize(numseeds);
	kseedsb.resize(numseeds);
	kseedsx.resize(numseeds);
	kseedsy.resize(numseeds);

	for( int y = 0; y < ystrips; y++ )
	{
		int ye = y*yerrperstrip;
		for( int x = 0; x < xstrips; x++ )
		{
			int xe = x*xerrperstrip;
            int seedx = (x*STEP+xoff+xe);
            if(hexgrid){ seedx = x*STEP+(xoff<<(y&0x1))+xe; seedx = min(m_width-1,seedx); }//for hex grid sampling
            int seedy = (y*STEP+yoff+ye);
            int i = seedy*m_width + seedx;
			
			kseedsl[n] = m_lvec[i];
			kseedsa[n] = m_avec[i];
			kseedsb[n] = m_bvec[i];
            kseedsx[n] = seedx;
            kseedsy[n] = seedy;
			n++;
		}
	}

	
	if(perturbseeds)
	{
		PerturbSeeds(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, edgemag);
	}
}

//===========================================================================
///	GetKValues_LABXYZ
///
/// The k seed values are taken as uniform spatial pixel samples.
//===========================================================================
void SLIC::GetKValues_LABXYZ(
	vector<double>&				kseedsl,
	vector<double>&				kseedsa,
	vector<double>&				kseedsb,
	vector<double>&				kseedsx,
	vector<double>&				kseedsy,
	vector<double>&				kseedsz,
        const int&				STEP)
{
    const bool hexgrid = false;
	int numseeds(0);
	int n(0);

	int xstrips = (0.5+double(m_width)/double(STEP));
	int ystrips = (0.5+double(m_height)/double(STEP));
	int zstrips = (0.5+double(m_depth)/double(STEP));

    int xerr = m_width  - STEP*xstrips;if(xerr < 0){xstrips--;xerr = m_width - STEP*xstrips;}
    int yerr = m_height - STEP*ystrips;if(yerr < 0){ystrips--;yerr = m_height- STEP*ystrips;}
    int zerr = m_depth  - STEP*zstrips;if(zerr < 0){zstrips--;zerr = m_depth - STEP*zstrips;}

	double xerrperstrip = double(xerr)/double(xstrips);
	double yerrperstrip = double(yerr)/double(ystrips);
	double zerrperstrip = double(zerr)/double(zstrips);

	int xoff = STEP/2;
	int yoff = STEP/2;
	int zoff = STEP/2;
	//-------------------------
	numseeds = xstrips*ystrips*zstrips;
	//-------------------------
	kseedsl.resize(numseeds);
	kseedsa.resize(numseeds);
	kseedsb.resize(numseeds);
	kseedsx.resize(numseeds);
	kseedsy.resize(numseeds);
	kseedsz.resize(numseeds);

	for( int z = 0; z < zstrips; z++ )
	{
		int ze = z*zerrperstrip;
		int d = (z*STEP+zoff+ze);
		for( int y = 0; y < ystrips; y++ )
		{
			int ye = y*yerrperstrip;
			for( int x = 0; x < xstrips; x++ )
			{
				int xe = x*xerrperstrip;
				int i = (y*STEP+yoff+ye)*m_width + (x*STEP+xoff+xe);
				
				kseedsl[n] = m_lvecvec[d][i];
				kseedsa[n] = m_avecvec[d][i];
				kseedsb[n] = m_bvecvec[d][i];
				kseedsx[n] = (x*STEP+xoff+xe);
				kseedsy[n] = (y*STEP+yoff+ye);
				kseedsz[n] = d;
				n++;
			}
		}
	}
}

//===========================================================================
///	PerformSuperpixelSLIC
///
///	Performs k mean segmentation. It is fast because it looks locally, not
/// over the entire image.
//===========================================================================
void SLIC::PerformSuperpixelSLIC(
	vector<double>&				kseedsl,
	vector<double>&				kseedsa,
	vector<double>&				kseedsb,
	vector<double>&				kseedsx,
	vector<double>&				kseedsy,
        int*&					klabels,
        const int&				STEP,
        const vector<double>&                   edgemag,
	const double&				M)
{
	int sz = m_width*m_height;
	const int numk = kseedsl.size();
	//----------------
	int offset = STEP;
        //if(STEP < 8) offset = STEP*1.5;//to prevent a crash due to a very small step size
	//----------------
	
	vector<double> clustersize(numk, 0);
	vector<double> inv(numk, 0);//to store 1/clustersize[k] values

	vector<double> sigmal(numk, 0);
	vector<double> sigmaa(numk, 0);
	vector<double> sigmab(numk, 0);
	vector<double> sigmax(numk, 0);
	vector<double> sigmay(numk, 0);
	vector<double> distvec(sz, DBL_MAX);

	double invwt = 1.0/((STEP/M)*(STEP/M));

	int x1, y1, x2, y2;
	double l, a, b;
	double dist;
	double distxy;
	for( int itr = 0; itr < 10; itr++ )
	{
		distvec.assign(sz, DBL_MAX);
		for( int n = 0; n < numk; n++ )
		{
                        y1 = max(0.0,			kseedsy[n]-offset);
                        y2 = min((double)m_height,	kseedsy[n]+offset);
                        x1 = max(0.0,			kseedsx[n]-offset);
                        x2 = min((double)m_width,	kseedsx[n]+offset);


			for( int y = y1; y < y2; y++ )
			{
				for( int x = x1; x < x2; x++ )
				{
					int i = y*m_width + x;

					l = m_lvec[i];
					a = m_avec[i];
					b = m_bvec[i];

					dist =			(l - kseedsl[n])*(l - kseedsl[n]) +
									(a - kseedsa[n])*(a - kseedsa[n]) +
									(b - kseedsb[n])*(b - kseedsb[n]);

					distxy =		(x - kseedsx[n])*(x - kseedsx[n]) +
									(y - kseedsy[n])*(y - kseedsy[n]);
					
					//------------------------------------------------------------------------
					dist += distxy*invwt;//dist = sqrt(dist) + sqrt(distxy*invwt);//this is more exact
					//------------------------------------------------------------------------
					if( dist < distvec[i] )
					{
						distvec[i] = dist;
						klabels[i]  = n;
					}
				}
			}
		}
		//-----------------------------------------------------------------
		// Recalculate the centroid and store in the seed values
		//-----------------------------------------------------------------
		//instead of reassigning memory on each iteration, just reset.
	
		sigmal.assign(numk, 0);
		sigmaa.assign(numk, 0);
		sigmab.assign(numk, 0);
		sigmax.assign(numk, 0);
		sigmay.assign(numk, 0);
		clustersize.assign(numk, 0);
		//------------------------------------
		//edgesum.assign(numk, 0);
		//------------------------------------

		{int ind(0);
		for( int r = 0; r < m_height; r++ )
		{
			for( int c = 0; c < m_width; c++ )
			{
				sigmal[klabels[ind]] += m_lvec[ind];
				sigmaa[klabels[ind]] += m_avec[ind];
				sigmab[klabels[ind]] += m_bvec[ind];
				sigmax[klabels[ind]] += c;
				sigmay[klabels[ind]] += r;
				//------------------------------------
				//edgesum[klabels[ind]] += edgemag[ind];
				//------------------------------------
				clustersize[klabels[ind]] += 1.0;
				ind++;
			}
		}}

		{for( int k = 0; k < numk; k++ )
		{
			if( clustersize[k] <= 0 ) clustersize[k] = 1;
			inv[k] = 1.0/clustersize[k];//computing inverse now to multiply, than divide later
		}}
		
		{for( int k = 0; k < numk; k++ )
		{
			kseedsl[k] = sigmal[k]*inv[k];
			kseedsa[k] = sigmaa[k]*inv[k];
			kseedsb[k] = sigmab[k]*inv[k];
			kseedsx[k] = sigmax[k]*inv[k];
			kseedsy[k] = sigmay[k]*inv[k];
			//------------------------------------
			//edgesum[k] *= inv[k];
			//------------------------------------
		}}
	}
}

//===========================================================================
///	PerformSupervoxelSLIC
///
///	Performs k mean segmentation. It is fast because it searches locally, not
/// over the entire image.
//===========================================================================
void SLIC::PerformSupervoxelSLIC(
	vector<double>&				kseedsl,
	vector<double>&				kseedsa,
	vector<double>&				kseedsb,
	vector<double>&				kseedsx,
	vector<double>&				kseedsy,
	vector<double>&				kseedsz,
        int**&					klabels,
        const int&				STEP,
	const double&				compactness)
{
	int sz = m_width*m_height;
	const int numk = kseedsl.size();
        //int numitr(0);

	//----------------
	int offset = STEP;
        //if(STEP < 8) offset = STEP*1.5;//to prevent a crash due to a very small step size
	//----------------

	vector<double> clustersize(numk, 0);
	vector<double> inv(numk, 0);//to store 1/clustersize[k] values

	vector<double> sigmal(numk, 0);
	vector<double> sigmaa(numk, 0);
	vector<double> sigmab(numk, 0);
	vector<double> sigmax(numk, 0);
	vector<double> sigmay(numk, 0);
	vector<double> sigmaz(numk, 0);

	vector< double > initdouble(sz, DBL_MAX);
	vector< vector<double> > distvec(m_depth, initdouble);
	//vector<double> distvec(sz, DBL_MAX);

	double invwt = 1.0/((STEP/compactness)*(STEP/compactness));//compactness = 20.0 is usually good.

	int x1, y1, x2, y2, z1, z2;
	double l, a, b;
	double dist;
	double distxyz;
	for( int itr = 0; itr < 5; itr++ )
	{
		distvec.assign(m_depth, initdouble);
		for( int n = 0; n < numk; n++ )
		{
                        y1 = max(0.0,			kseedsy[n]-offset);
                        y2 = min((double)m_height,	kseedsy[n]+offset);
                        x1 = max(0.0,			kseedsx[n]-offset);
                        x2 = min((double)m_width,	kseedsx[n]+offset);
                        z1 = max(0.0,			kseedsz[n]-offset);
                        z2 = min((double)m_depth,	kseedsz[n]+offset);


			for( int z = z1; z < z2; z++ )
			{
				for( int y = y1; y < y2; y++ )
				{
					for( int x = x1; x < x2; x++ )
					{
						int i = y*m_width + x;

						l = m_lvecvec[z][i];
						a = m_avecvec[z][i];
						b = m_bvecvec[z][i];

						dist =			(l - kseedsl[n])*(l - kseedsl[n]) +
										(a - kseedsa[n])*(a - kseedsa[n]) +
										(b - kseedsb[n])*(b - kseedsb[n]);

						distxyz =		(x - kseedsx[n])*(x - kseedsx[n]) +
										(y - kseedsy[n])*(y - kseedsy[n]) +
										(z - kseedsz[n])*(z - kseedsz[n]);
						//------------------------------------------------------------------------
						dist += distxyz*invwt;
						//------------------------------------------------------------------------
						if( dist < distvec[z][i] )
						{
							distvec[z][i] = dist;
							klabels[z][i]  = n;
						}
					}
				}
			}
		}
		//-----------------------------------------------------------------
		// Recalculate the centroid and store in the seed values
		//-----------------------------------------------------------------
		//instead of reassigning memory on each iteration, just reset.
	
		sigmal.assign(numk, 0);
		sigmaa.assign(numk, 0);
		sigmab.assign(numk, 0);
		sigmax.assign(numk, 0);
		sigmay.assign(numk, 0);
		sigmaz.assign(numk, 0);
		clustersize.assign(numk, 0);

		for( int d = 0; d < m_depth; d++  )
		{
			int ind(0);
			for( int r = 0; r < m_height; r++ )
			{
				for( int c = 0; c < m_width; c++ )
				{
					sigmal[klabels[d][ind]] += m_lvecvec[d][ind];
					sigmaa[klabels[d][ind]] += m_avecvec[d][ind];
					sigmab[klabels[d][ind]] += m_bvecvec[d][ind];
					sigmax[klabels[d][ind]] += c;
					sigmay[klabels[d][ind]] += r;
					sigmaz[klabels[d][ind]] += d;

					clustersize[klabels[d][ind]] += 1.0;
					ind++;
				}
			}
		}

		{for( int k = 0; k < numk; k++ )
		{
			if( clustersize[k] <= 0 ) clustersize[k] = 1;
			inv[k] = 1.0/clustersize[k];//computing inverse now to multiply, than divide later
		}}
		
		{for( int k = 0; k < numk; k++ )
		{
			kseedsl[k] = sigmal[k]*inv[k];
			kseedsa[k] = sigmaa[k]*inv[k];
			kseedsb[k] = sigmab[k]*inv[k];
			kseedsx[k] = sigmax[k]*inv[k];
			kseedsy[k] = sigmay[k]*inv[k];
			kseedsz[k] = sigmaz[k]*inv[k];
		}}
	}
}


//===========================================================================
///	SaveSuperpixelLabels
///
///	Save labels in raster scan order.
//===========================================================================
void SLIC::SaveSuperpixelLabels(
	const int*					labels,
	const int&					width,
	const int&					height,
	const string&				filename,
	const string&				path) 
{
//#ifdef WINDOWS
        char fname[256];
        char extn[256];
        _splitpath(filename.c_str(), NULL, NULL, fname, extn);
        string temp = fname;
        string finalpath = path + temp + string(".dat");
/*#else
        string nameandextension = filename;
        size_t pos = filename.find_last_of("/");
        if(pos != string::npos)//if a slash is found, then take the filename with extension
        {
            nameandextension = filename.substr(pos+1);
        }
        string newname = nameandextension.replace(nameandextension.rfind(".")+1, 3, "dat");//find the position of the dot and replace the 3 characters following it.
        string finalpath = path+newname;
#endif*/

        int sz = width*height;
	ofstream outfile;
	outfile.open(finalpath.c_str(), ios::binary);
	for( int i = 0; i < sz; i++ )
	{
		outfile.write((const char*)&labels[i], sizeof(int));
	}
	outfile.close();
}


//===========================================================================
///	SaveSupervoxelLabels
///
///	Save labels in raster scan order.
//===========================================================================
void SLIC::SaveSupervoxelLabels(
	const int**&				labels,
	const int&					width,
	const int&					height,
	const int&					depth,
	const string&				filename,
	const string&				path) 
{
#ifdef WINDOWS
        char fname[256];
        char extn[256];
        _splitpath(filename.c_str(), NULL, NULL, fname, extn);
        string temp = fname;
        string finalpath = path + temp + string(".dat");
#else
        string nameandextension = filename;
        size_t pos = filename.find_last_of("/");
        if(pos != string::npos)//if a slash is found, then take the filename with extension
        {
            nameandextension = filename.substr(pos+1);
        }
        string newname = nameandextension.replace(nameandextension.rfind(".")+1, 3, "dat");//find the position of the dot and replace the 3 characters following it.
        string finalpath = path+newname;
#endif

        int sz = width*height;
	ofstream outfile;
	outfile.open(finalpath.c_str(), ios::binary);
	for( int d = 0; d < depth; d++ )
	{
		for( int i = 0; i < sz; i++ )
		{
			outfile.write((const char*)&labels[d][i], sizeof(int));
		}
	}
	outfile.close();
}

//===========================================================================
///	EnforceLabelConnectivity
///
///		1. finding an adjacent label for each new component at the start
///		2. if a certain component is too small, assigning the previously found
///		    adjacent label to this component, and not incrementing the label.
//===========================================================================
void SLIC::EnforceLabelConnectivity(
	const int*					labels,//input labels that need to be corrected to remove stray labels
	const int					width,
	const int					height,
	int*&						nlabels,//new labels
	int&						numlabels,//the number of labels changes in the end if segments are removed
	const int&					K) //the number of superpixels desired by the user
{
//	const int dx8[8] = {-1, -1,  0,  1, 1, 1, 0, -1};
//	const int dy8[8] = { 0, -1, -1, -1, 0, 1, 1,  1};

	const int dx4[4] = {-1,  0,  1,  0};
	const int dy4[4] = { 0, -1,  0,  1};

	const int sz = width*height;
	const int SUPSZ = sz/K;
	//nlabels.resize(sz, -1);
	for( int i = 0; i < sz; i++ ) nlabels[i] = -1;
	int label(0);
	int* xvec = new int[sz];
	int* yvec = new int[sz];
	int oindex(0);
	int adjlabel(0);//adjacent label
	for( int j = 0; j < height; j++ )
	{
		for( int k = 0; k < width; k++ )
		{
			if( 0 > nlabels[oindex] )
			{
				nlabels[oindex] = label;
				//--------------------
				// Start a new segment
				//--------------------
				xvec[0] = k;
				yvec[0] = j;
				//-------------------------------------------------------
				// Quickly find an adjacent label for use later if needed
				//-------------------------------------------------------
				{for( int n = 0; n < 4; n++ )
				{
					int x = xvec[0] + dx4[n];
					int y = yvec[0] + dy4[n];
					if( (x >= 0 && x < width) && (y >= 0 && y < height) )
					{
						int nindex = y*width + x;
						if(nlabels[nindex] >= 0) adjlabel = nlabels[nindex];
					}
				}}

				int count(1);
				for( int c = 0; c < count; c++ )
				{
					for( int n = 0; n < 4; n++ )
					{
						int x = xvec[c] + dx4[n];
						int y = yvec[c] + dy4[n];

						if( (x >= 0 && x < width) && (y >= 0 && y < height) )
						{
							int nindex = y*width + x;

							if( 0 > nlabels[nindex] && labels[oindex] == labels[nindex] )
							{
								xvec[count] = x;
								yvec[count] = y;
								nlabels[nindex] = label;
								count++;
							}
						}

					}
				}
				//-------------------------------------------------------
				// If segment size is less then a limit, assign an
				// adjacent label found before, and decrement label count.
				//-------------------------------------------------------
				if(count <= SUPSZ >> 2)
				{
					for( int c = 0; c < count; c++ )
					{
						int ind = yvec[c]*width+xvec[c];
						nlabels[ind] = adjlabel;
					}
					label--;
				}
				label++;
			}
			oindex++;
		}
	}
	numlabels = label;

	if(xvec) delete [] xvec;
	if(yvec) delete [] yvec;
}


//===========================================================================
///	RelabelStraySupervoxels
//===========================================================================
void SLIC::EnforceSupervoxelLabelConnectivity(
	int**&						labels,//input - previous labels, output - new labels
	const int&					width,
	const int&					height,
	const int&					depth,
	int&						numlabels,
	const int&					STEP)
{
	const int dx10[10] = {-1,  0,  1,  0, -1,  1,  1, -1,  0, 0};
	const int dy10[10] = { 0, -1,  0,  1, -1, -1,  1,  1,  0, 0};
	const int dz10[10] = { 0,  0,  0,  0,  0,  0,  0,  0, -1, 1};

	int sz = width*height;
	const int SUPSZ = STEP*STEP*STEP;

	int adjlabel(0);//adjacent label
        int* xvec = new int[SUPSZ*10];//a large enough size
        int* yvec = new int[SUPSZ*10];//a large enough size
        int* zvec = new int[SUPSZ*10];//a large enough size
	//------------------
	// memory allocation
	//------------------
	int** nlabels = new int*[depth];
	{for( int d = 0; d < depth; d++ )
	{
		nlabels[d] = new int[sz];
		for( int i = 0; i < sz; i++ ) nlabels[d][i] = -1;
	}}
	//------------------
	// labeling
	//------------------
	int lab(0);
	{for( int d = 0; d < depth; d++ )
	{
		int i(0);
		for( int h = 0; h < height; h++ )
		{
			for( int w = 0; w < width; w++ )
			{
				if(nlabels[d][i] < 0)
				{
					nlabels[d][i] = lab;
					//-------------------------------------------------------
					// Quickly find an adjacent label for use later if needed
					//-------------------------------------------------------
					{for( int n = 0; n < 10; n++ )
					{
						int x = w + dx10[n];
						int y = h + dy10[n];
						int z = d + dz10[n];
						if( (x >= 0 && x < width) && (y >= 0 && y < height) && (z >= 0 && z < depth) )
						{
							int nindex = y*width + x;
							if(nlabels[z][nindex] >= 0)
							{
								adjlabel = nlabels[z][nindex];
							}
						}
					}}
					
					xvec[0] = w; yvec[0] = h; zvec[0] = d;
					int count(1);
					for( int c = 0; c < count; c++ )
					{
						for( int n = 0; n < 10; n++ )
						{
							int x = xvec[c] + dx10[n];
							int y = yvec[c] + dy10[n];
							int z = zvec[c] + dz10[n];

							if( (x >= 0 && x < width) && (y >= 0 && y < height) && (z >= 0 && z < depth))
							{
								int nindex = y*width + x;

								if( 0 > nlabels[z][nindex] && labels[d][i] == labels[z][nindex] )
								{
									xvec[count] = x;
									yvec[count] = y;
									zvec[count] = z;
									nlabels[z][nindex] = lab;
									count++;
								}
							}

						}
					}
					//-------------------------------------------------------
					// If segment size is less then a limit, assign an
					// adjacent label found before, and decrement label count.
					//-------------------------------------------------------
					if(count <= (SUPSZ >> 2))//this threshold can be changed according to needs
					{
						for( int c = 0; c < count; c++ )
						{
							int ind = yvec[c]*width+xvec[c];
							nlabels[zvec[c]][ind] = adjlabel;
						}
						lab--;
					}
					//--------------------------------------------------------
					lab++;
				}
				i++;
			}
		}
	}}
	//------------------
	// mem de-allocation
	//------------------
	{for( int d = 0; d < depth; d++ )
	{
		for( int i = 0; i < sz; i++ ) labels[d][i] = nlabels[d][i];
	}}
	{for( int d = 0; d < depth; d++ )
	{
		delete [] nlabels[d];
	}}
	delete [] nlabels;
	//------------------
	if(xvec) delete [] xvec;
	if(yvec) delete [] yvec;
	if(zvec) delete [] zvec;
	//------------------
	numlabels = lab;
	//------------------
}

//===========================================================================
///	DoSuperpixelSegmentation_ForGivenSuperpixelSize
///
/// The input parameter ubuff conains RGB values in a 32-bit unsigned integers
/// as follows:
///
/// [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]
///
///        Nothing              R                 G                  B
///
/// The RGB values are accessed from (and packed into) the unsigned integers
/// using bitwise operators as can be seen in the function DoRGBtoLABConversion().
///
/// compactness value depends on the input pixels values. For instance, if
/// the input is greyscale with values ranging from 0-100, then a compactness
/// value of 20.0 would give good results. A greater value will make the
/// superpixels more compact while a smaller value would make them more uneven.
///
/// The labels can be saved if needed using SaveSuperpixelLabels()
//===========================================================================
void SLIC::DoSuperpixelSegmentation_ForGivenSuperpixelSize(
    const unsigned int*         ubuff,
	const int					width,
	const int					height,
	int*&						klabels,
	int&						numlabels,
    const int&					superpixelsize,
    const double&               compactness)
{
    //------------------------------------------------
    const int STEP = sqrt(double(superpixelsize))+0.5;
    //------------------------------------------------
	vector<double> kseedsl(0);
	vector<double> kseedsa(0);
	vector<double> kseedsb(0);
	vector<double> kseedsx(0);
	vector<double> kseedsy(0);

	//--------------------------------------------------
	m_width  = width;
	m_height = height;
	int sz = m_width*m_height;
	//klabels.resize( sz, -1 );
	//--------------------------------------------------
	klabels = new int[sz];
	for( int s = 0; s < sz; s++ ) klabels[s] = -1;
    //--------------------------------------------------
    if(1)//LAB, the default option
    {
        DoRGBtoLABConversion(ubuff, m_lvec, m_avec, m_bvec);
    }
    else//RGB
    {
        m_lvec = new double[sz]; m_avec = new double[sz]; m_bvec = new double[sz];
        for( int i = 0; i < sz; i++ )
        {
                m_lvec[i] = ubuff[i] >> 16 & 0xff;
                m_avec[i] = ubuff[i] >>  8 & 0xff;
                m_bvec[i] = ubuff[i]       & 0xff;
        }
    }
	//--------------------------------------------------
    bool perturbseeds(false);//perturb seeds is not absolutely necessary, one can set this flag to false
	vector<double> edgemag(0);
	if(perturbseeds) DetectLabEdges(m_lvec, m_avec, m_bvec, m_width, m_height, edgemag);
	GetLABXYSeeds_ForGivenStepSize(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, STEP, perturbseeds, edgemag);

	PerformSuperpixelSLIC(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, klabels, STEP, edgemag,compactness);
	numlabels = kseedsl.size();

	int* nlabels = new int[sz];
	EnforceLabelConnectivity(klabels, m_width, m_height, nlabels, numlabels, double(sz)/double(STEP*STEP));
	{for(int i = 0; i < sz; i++ ) klabels[i] = nlabels[i];}
	if(nlabels) delete [] nlabels;
}

//===========================================================================
///	DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels
///
/// The input parameter ubuff conains RGB values in a 32-bit unsigned integers
/// as follows:
///
/// [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]
///
///        Nothing              R                 G                  B
///
/// The RGB values are accessed from (and packed into) the unsigned integers
/// using bitwise operators as can be seen in the function DoRGBtoLABConversion().
///
/// compactness value depends on the input pixels values. For instance, if
/// the input is greyscale with values ranging from 0-100, then a compactness
/// value of 20.0 would give good results. A greater value will make the
/// superpixels more compact while a smaller value would make them more uneven.
///
/// The labels can be saved if needed using SaveSuperpixelLabels()
//===========================================================================
void SLIC::DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(
    const unsigned int*                             ubuff,
	const int					width,
	const int					height,
	int*&						klabels,
	int&						numlabels,
	const int&					K,//required number of superpixels
    const double&                                   compactness)//weight given to spatial distance
{
    const int superpixelsize = 0.5+double(width*height)/double(K);
    DoSuperpixelSegmentation_ForGivenSuperpixelSize(ubuff,width,height,klabels,numlabels,superpixelsize,compactness);
}

//===========================================================================
///	DoSupervoxelSegmentation
///
/// There is option to save the labels if needed.
///
/// The input parameter ubuffvec holds all the video frames. It is a
/// 2-dimensional array. The first dimension is depth and the second dimension
/// is pixel location in a frame. For example, to access a pixel in the 3rd
/// frame (i.e. depth index 2), in the 4th row (i.e. height index 3) on the
/// 37th column (i.e. width index 36), you would write:
///
/// unsigned int the_pixel_i_want = ubuffvec[2][3*width + 36]
///
/// In addition, here is how the RGB values are contained in a 32-bit unsigned
/// integer:
///
/// [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]  [1 1 1 1 1 1 1 1]
///
///        Nothing              R                 G                  B
///
/// The RGB values are accessed from (and packed into) the unsigned integers
/// using bitwise operators as can be seen in the function DoRGBtoLABConversion().
///
/// compactness value depends on the input pixels values. For instance, if
/// the input is greyscale with values ranging from 0-100, then a compactness
/// value of 20.0 would give good results. A greater value will make the
/// supervoxels more compact while a smaller value would make them more uneven.
//===========================================================================
void SLIC::DoSupervoxelSegmentation(
	unsigned int**&				ubuffvec,
	const int&					width,
	const int&					height,
	const int&					depth,
	int**&						klabels,
	int&						numlabels,
    const int&					supervoxelsize,
    const double&               compactness)
{
    //---------------------------------------------------------
    const int STEP = 0.5 + pow(double(supervoxelsize),1.0/3.0);
    //---------------------------------------------------------
	vector<double> kseedsl(0);
	vector<double> kseedsa(0);
	vector<double> kseedsb(0);
	vector<double> kseedsx(0);
	vector<double> kseedsy(0);
	vector<double> kseedsz(0);

	//--------------------------------------------------
	m_width  = width;
	m_height = height;
	m_depth  = depth;
	int sz = m_width*m_height;
	
	//--------------------------------------------------
        //klabels = new int*[depth];
	m_lvecvec = new double*[depth];
	m_avecvec = new double*[depth];
	m_bvecvec = new double*[depth];
	for( int d = 0; d < depth; d++ )
	{
                //klabels[d] = new int[sz];
		m_lvecvec[d] = new double[sz];
		m_avecvec[d] = new double[sz];
		m_bvecvec[d] = new double[sz];
		for( int s = 0; s < sz; s++ )
		{
			klabels[d][s] = -1;
		}
	}
	
	DoRGBtoLABConversion(ubuffvec, m_lvecvec, m_avecvec, m_bvecvec);

	GetKValues_LABXYZ(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, kseedsz, STEP);

	PerformSupervoxelSLIC(kseedsl, kseedsa, kseedsb, kseedsx, kseedsy, kseedsz, klabels, STEP, compactness);

	EnforceSupervoxelLabelConnectivity(klabels, width, height, depth, numlabels, STEP);
}

