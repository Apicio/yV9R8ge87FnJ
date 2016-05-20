#ifndef CLASSIFICATION_H
#define CLASSIFICATION_H

#include <opencv2/opencv.hpp>
#include <vector>

using namespace std;
using namespace cv;

#define CLASSIFIER something
#define FEAT_SIZE 123

/**
 * @brief Extracts the feature vector form image
 * @param image Raw image to extract
 * @param featureVector A row vector of features
 */
void feature(Mat image, Mat& featureVector);

/**
 * @brief Trains the classifier on a data set
 * @param classifier The classifier to train
 * @param trainingFeatures The set of extracted feature (each row is a sample)
 * @param classLabels The label for each image
 */
void train(CLASSIFIER& classifier, Mat trainingFeatures, vector<int> classLabels);

/**
 * @brief Classifies a ROI
 * @param classifier The classifier to use
 * @param image ROI of the detection to classify
 * @return The class label
 */
int classify(CLASSIFIER& classifier, Mat image);

#endif // CLASSIFICATION_H
