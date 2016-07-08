#include "classification.h"
#include <algorithm>

void extractFeatures(Mat image, Mat& featureVector)
{
    /* Features are: the 7 Hu moments + average hue */
    Mat grayImage, hsvImage;
    cvtColor(image, grayImage, CV_BGR2GRAY);
    cvtColor(image, hsvImage, CV_BGR2HSV);
    float meanHue = mean(hsvImage)[0];
    Moments m = moments(grayImage);
    HuMoments(m, featureVector.colRange(0, 6));
    featureVector.at<float>(0, 7) = meanHue / 360;
}

void trainMulticlassSvm(CvSVM& classifier, Mat trainingFeatures, vector<int> classLabels, int kFolds)
{
    /* Model parameters */
    CvSVMParams params;
    params.kernel_type = CvSVM::LINEAR;
    params.svm_type = CvSVM::C_SVC;

    /* Leave-one-out or k-folds crossvalidation? */
    if (kFolds <= 0) {
        kFolds = trainingFeatures.rows;
    }

    /* The SVM wants class labels as floats (weird) */
    Mat classes(classLabels.size(), 1, CV_32FC1);
    for (size_t i = 0; i < classLabels.size(); i++) {
        classes.at<float>(i, 0) = classLabels[i];
    }

    /* Now train the shit out of it */
    classifier.train_auto(trainingFeatures, classes, Mat(), Mat(), params, kFolds);
}

int classifyMulticlassSvm(CvSVM& classifier, Mat image)
{
    /* Extract features */
    Mat featureVector(FEAT_SIZE, 1, CV_32FC1);
    extractFeatures(image, featureVector);

    /* Classify */
    float classLabel = classifier.predict(featureVector, true);

    /* Integer class label */
    return (int)classLabel;
}

void trainOneAgainstAllSvm(vector<CvSVM> classifiers, Mat trainingFeatures, vector<int> classLabels, int kFolds)
{
    /* Model parameters */
    CvSVMParams params;
    params.kernel_type = CvSVM::LINEAR;
    params.svm_type = CvSVM::C_SVC;
    //params.svm_type = CvSVM::ONE_CLASS;

    /* Leave-one-out or k-folds crossvalidation? */
    if (kFolds <= 0) {
        kFolds = trainingFeatures.rows;
    }

    /* Train each classifier */
    for (int i = 0; i < classifiers.size(); i++) {

        /* The label is +1 for samples in class, -1 for others */
        Mat labels(classLabels.size(), 1, CV_32FC1);
        for (int j = 0; j < classLabels.size(); j++) {
            labels.at<float>(j, 0) = (classLabels[j] == i+1) ? +1 : -1;
        }

        /* Now train the shit out of it */
        classifiers[i].train_auto(trainingFeatures, labels, Mat(), Mat(), params, kFolds);
    }
}

int classifyOneAgainstAllSvm(vector<CvSVM> classifiers, Mat image)
{
    /* Extract features */
    Mat featureVector(FEAT_SIZE, 1, CV_32FC1);
    extractFeatures(image, featureVector);

    /* Responses from the classifiers */
    vector<float> responses(classifiers.size());
    for (int i = 0; i < classifiers.size(); i++) {
        responses[i] = classifiers[i].predict(featureVector, true);
    }

    /* Argmax of responses */
    int argmax = 0;
    for (int i = 1; i < responses.size(); i++) {
        if (responses[i] > responses[argmax]) {
            argmax = i;
        }
    }

    /* Got a label or a rejection? */
    return (responses[argmax] > 0) ? (argmax+1) : 0;
}
