#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

#define FEAT_SIZE 8

/**
 * @brief Extracts the feature vector form image
 * @param image Raw image to extract
 * @param featureVector A row vector of features
 */
void extractFeatures(Mat image, Mat& featureVector);

/**
 * @brief Trains a single multiclass SVM classifier on a data set
 * @param classifier The classifier to train
 * @param trainingFeatures The set of extracted feature (each row is a sample)
 * @param classLabels The label for each image
 * @param kFolds Uses k-folds crossvalidation (default is leave-one-out)
 */
void trainMulticlassSvm(CvSVM& classifier, Mat trainingFeatures, vector<int> classLabels, int kFolds = 0);

/**
 * @brief Classifies a ROI with a single multiclass SVM
 * @param classifier The classifier to use
 * @param image ROI of the detection to classify
 * @return The class label
 */
int classifyMulticlassSvm(CvSVM& classifier, Mat image);

/**
 * @brief Trains multiple single-class SVM classifiers on a data set
 * @param classifiers The classifiers to train (N, one for class)
 * @param trainingFeatures The set of extracted feature (each row is a sample)
 * @param classLabels The label for each image (1 to N)
 * @param kFolds Uses k-folds crossvalidation (default is leave-one-out)
 */
void trainOneAgainstAllSvm(vector<CvSVM> classifiers, Mat trainingFeatures, vector<int> classLabels, int kFolds = 0);

/**
 * @brief Classifies a ROI with multiple single-class SVM classifiers
 * @param classifier The classifiers to use
 * @param image ROI of the detection to classify
 * @return The class label, or 0 for rejection
 */
int classifyOneAgainstAllSvm(vector<CvSVM> classifiers, Mat image);

#endif // CLASSIFICATION_H
