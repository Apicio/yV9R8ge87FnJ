// SLIC.h: interface for the SLIC class.
//===========================================================================
// This code implements the superpixel method described in:
//
// Radhakrishna Achanta, Appu Shaji, Kevin Smith, Aurelien Lucchi, Pascal Fua, and Sabine Susstrunk,
// "SLIC Superpixels",
// EPFL Technical Report no. 149300, June 2010.
//===========================================================================
//	Copyright (c) 2012 Radhakrishna Achanta [EPFL]. All rights reserved.
//===========================================================================
//////////////////////////////////////////////////////////////////////

#if !defined(_SLIC_H_INCLUDED_)
#define _SLIC_H_INCLUDED_


#include <vector>
#include <string>
#include <algorithm>
#include <opencv2/opencv.hpp>
#include "Constants.h"
#include "detection.h"
using namespace std;

struct SuperPixel{
	vector<cv::Point > points;
	vector<int> nearests;
	vector<int> similars;
	cv::Mat superpixel;
	cv::Mat hist_base;
	cv::Mat hist_orig;
	int label;
	bool isOfInterest;
};


class SLIC  
{
public:
	SLIC();
	virtual ~SLIC();
	void SLIC::mergeSuperPixel(SuperPixel & a,SuperPixel & b);
	void SLIC::GetPixelsSet(
	cv::Mat 			img,
	const int*				labels,
	const int&				width,
	const int&				height,
	int numlabels,
	vector<SuperPixel > &set,
	bool**& matrix);
	//============================================================================
	// Superpixel segmentation for a given step size (superpixel size ~= step*step)
	//============================================================================
        void DoSuperpixelSegmentation_ForGivenSuperpixelSize(
        const unsigned int*                            ubuff,//Each 32 bit unsigned int contains ARGB pixel values.
		const int					width,
		const int					height,
		int*&						klabels,
		int&						numlabels,
                const int&					superpixelsize,
                const double&                                   compactness);
	//============================================================================
	// Superpixel segmentation for a given number of superpixels
	//============================================================================
        void DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(
        const unsigned int*                             ubuff,
		const int					width,
		const int					height,
		int*&						klabels,
		int&						numlabels,
                const int&					K,//required number of superpixels
                const double&                                   compactness);//10-20 is a good value for CIELAB space
	//============================================================================
	// Supervoxel segmentation for a given step size (supervoxel size ~= step*step*step)
	//============================================================================
	void DoSupervoxelSegmentation(
		unsigned int**&		ubuffvec,
		const int&					width,
		const int&					height,
		const int&					depth,
		int**&						klabels,
		int&						numlabels,
                const int&					supervoxelsize,
                const double&                                   compactness);
	//============================================================================
	// Save superpixel labels in a text file in raster scan order
	//============================================================================
	void SaveSuperpixelLabels(
		const int*					labels,
		const int&					width,
		const int&					height,
		const string&				filename,
		const string&				path);
	//============================================================================
	// Save supervoxel labels in a text file in raster scan, depth order
	//============================================================================
	void SaveSupervoxelLabels(
		const int**&				labels,
		const int&					width,
		const int&					height,
		const int&					depth,
		const string&				filename,
		const string&				path);
	//============================================================================
	// Function to draw boundaries around superpixels of a given 'color'.
	// Can also be used to draw boundaries around supervoxels, i.e layer by layer.
	//============================================================================
	void DrawContoursAroundSegments(
		unsigned int*&				segmentedImage,
		int*&						labels,
		const int&					width,
		const int&					height,
		const unsigned int&			color );
	//============================================================================
	// Function to draw boundaries around superpixels of a given 'color'.
	// Can also be used to draw boundaries around supervoxels, i.e layer by layer.
	//============================================================================
	void DrawContoursAroundSegments(
		unsigned int*				ubuff,
		const int*					labels,
		const int&					width,
		const int&					height,
		const cv::Scalar&			color );

	void DrawContoursAroundSegments(
		unsigned char*			ubuff,
		const int*				labels,
		const int&				width,
		const int&				height,
		const cv::Scalar&		color );

	void DrawContoursAroundSegmentsTwoColors(
		unsigned int*				ubuff,
		const int*					labels,
		const int&					width,
		const int&					height);

private:
	//============================================================================
	// The main SLIC algorithm for generating superpixels
	//============================================================================
	void PerformSuperpixelSLIC(
		vector<double>&				kseedsl,
		vector<double>&				kseedsa,
		vector<double>&				kseedsb,
		vector<double>&				kseedsx,
		vector<double>&				kseedsy,
		int*&						klabels,
		const int&					STEP,
                const vector<double>&		edgemag,
		const double&				m = 10.0);
	//============================================================================
	// The main SLIC algorithm for generating supervoxels
	//============================================================================
	void PerformSupervoxelSLIC(
		vector<double>&				kseedsl,
		vector<double>&				kseedsa,
		vector<double>&				kseedsb,
		vector<double>&				kseedsx,
		vector<double>&				kseedsy,
		vector<double>&				kseedsz,
		int**&						klabels,
		const int&					STEP,
		const double&				compactness);
	//============================================================================
	// Pick seeds for superpixels when step size of superpixels is given.
	//============================================================================
	void GetLABXYSeeds_ForGivenStepSize(
		vector<double>&				kseedsl,
		vector<double>&				kseedsa,
		vector<double>&				kseedsb,
		vector<double>&				kseedsx,
		vector<double>&				kseedsy,
		const int&					STEP,
		const bool&					perturbseeds,
		const vector<double>&		edgemag);
	//============================================================================
	// Pick seeds for supervoxels
	//============================================================================
	void GetKValues_LABXYZ(
		vector<double>&				kseedsl,
		vector<double>&				kseedsa,
		vector<double>&				kseedsb,
		vector<double>&				kseedsx,
		vector<double>&				kseedsy,
		vector<double>&				kseedsz,
		const int&					STEP);
	//============================================================================
	// Move the superpixel seeds to low gradient positions to avoid putting seeds
	// at region boundaries.
	//============================================================================
	void PerturbSeeds(
		vector<double>&				kseedsl,
		vector<double>&				kseedsa,
		vector<double>&				kseedsb,
		vector<double>&				kseedsx,
		vector<double>&				kseedsy,
		const vector<double>&		edges);
	//============================================================================
	// Detect color edges, to help PerturbSeeds()
	//============================================================================
	void DetectLabEdges(
		const double*				lvec,
		const double*				avec,
		const double*				bvec,
		const int&					width,
		const int&					height,
		vector<double>&				edges);
	//============================================================================
	// sRGB to XYZ conversion; helper for RGB2LAB()
	//============================================================================
	void RGB2XYZ(
		const int&					sR,
		const int&					sG,
		const int&					sB,
		double&						X,
		double&						Y,
		double&						Z);
	//============================================================================
	// sRGB to CIELAB conversion (uses RGB2XYZ function)
	//============================================================================
	void RGB2LAB(
		const int&					sR,
		const int&					sG,
		const int&					sB,
		double&						lval,
		double&						aval,
		double&						bval);
	//============================================================================
	// sRGB to CIELAB conversion for 2-D images
	//============================================================================
	void DoRGBtoLABConversion(
		const unsigned int*&		ubuff,
		double*&					lvec,
		double*&					avec,
		double*&					bvec);
	//============================================================================
	// sRGB to CIELAB conversion for 3-D volumes
	//============================================================================
	void DoRGBtoLABConversion(
		unsigned int**&				ubuff,
		double**&					lvec,
		double**&					avec,
		double**&					bvec);
	//============================================================================
	// Post-processing of SLIC segmentation, to avoid stray labels.
	//============================================================================
	void EnforceLabelConnectivity(
		const int*					labels,
		const int					width,
		const int					height,
		int*&						nlabels,//input labels that need to be corrected to remove stray labels
		int&						numlabels,//the number of labels changes in the end if segments are removed
		const int&					K); //the number of superpixels desired by the user
	//============================================================================
	// Post-processing of SLIC supervoxel segmentation, to avoid stray labels.
	//============================================================================
	void EnforceSupervoxelLabelConnectivity(
		int**&						labels,//input - previous labels, output - new labels
		const int&					width,
		const int&					height,
		const int&					depth,
		int&						numlabels,
		const int&					STEP);

private:
	int										m_width;
	int										m_height;
	int										m_depth;

	double*									m_lvec;
	double*									m_avec;
	double*									m_bvec;

	double**								m_lvecvec;
	double**								m_avecvec;
	double**								m_bvecvec;
};

#endif // !defined(_SLIC_H_INCLUDED_)




#if 0 //BACKUP

	cv::Mat img;
for(int i=1;i<121;i++){
		stringstream img_file;
		img_file<<"DatasetLippi\\im ("<<i<<").jpg";
		img = imread(img_file.str());
int width = img.cols;
int height = img.rows;
resize(img, img, Size(width/2,height/2), 0, 0, INTER_NEAREST);
width = img.cols;
height = img.rows;
int sz = width*height;
int m_compactness = 10;
int m_spcount = 200;
int numlabels = 20;
int* labels = new int[sz];
SLIC slic;
//---------------------------------------------------------
if(m_spcount < 20 || m_spcount > sz/4) m_spcount = sz/200;//i.e the default size of the superpixel is 200 pixels
if(m_compactness < 1.0 || m_compactness > 80.0) m_compactness = 20.0;
//---------------------------------------------------------

UINT* imgBuffer = new UINT[sz];
cv::Mat newImage;
cv::cvtColor(img, newImage, CV_BGR2BGRA);
memcpy( imgBuffer, (UINT*)newImage.data, sz*sizeof(UINT) );

slic.DoSuperpixelSegmentation_ForGivenNumberOfSuperpixels(imgBuffer, width, height, labels, numlabels, m_spcount, m_compactness);
//slic.DoSuperpixelSegmentation_ForGivenSuperpixelSize(img, width, height, labels, numlabels, 10, m_compactness);//demo
slic.SaveSuperpixelLabels(labels,width,height,"fuck.txt","C:\\Users\\leo\\Desktop\\NaoVision\\"); 
slic.DrawContoursAroundSegments(imgBuffer, labels, width, height, Scalar(0,0));
vector<vector<cv::Point > > set;
slic.GetPixelsSet(imgBuffer, labels, width, height, numlabels, set);
//if(labels) delete [] labels;
cv::Mat result(height, width, CV_8UC4);
memcpy(result.data, imgBuffer, sz*sizeof(UINT));
cvtColor(result, result, CV_BGRA2BGR);
//picHand.SavePicture(imgBuffer, width, height, picvec[k], saveLocation, 1, "_SLIC");// 0 is for BMP and 1 for JPEG)
//if(img) delete [] img;
imshow("abc",result);
waitKey(1);

Mat bands[3];
split(result,bands);
Mat mask1 = bands[0] == 0;
Mat mask2 = bands[1] == 0;
Mat mask3 = bands[2] == 0;
Mat mask4 = Mat::zeros(img.size(),CV_8U);
for(int idx = 0; idx < mask4.rows; idx++){
	mask4.at<uchar>(idx,0) = 255;
	mask4.at<uchar>(idx,1) = 255;
	mask4.at<uchar>(idx,mask4.cols-1) = 255;
	mask4.at<uchar>(idx,mask4.cols-2) = 255;
}
for(int idx = 0; idx < mask4.cols; idx++){
	mask4.at<uchar>(0,idx) = 255;
	mask4.at<uchar>(1,idx) = 255;
	mask4.at<uchar>(mask4.rows-1,idx) = 255;
	mask4.at<uchar>(mask4.rows-2,idx) = 255;
}
bitwise_and(mask1,  mask2, mask2);
bitwise_and(mask3,  mask2, mask3);
mask4 = mask3 + mask4;
imshow("abcd",mask4);
waitKey(1);
vector< vector<Point> > contours;
findContours(mask4, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
Mat toReturn = img.clone();
for(int uu = 0; uu<contours.size(); uu++){
	drawContours(toReturn,contours,uu,Scalar(rand() % 256,rand() % 256,rand() % 256),CV_FILLED);
}
imshow("abcde",toReturn);
waitKey(1);
cout << i << endl;
}
#endif