#include "classification.h"

void feature(Mat image, Mat& featureVector)
{
    /* Features are: the 7 Hu moments + average hue */
    Mat grayImage, hsvImage;
    cvtColor(image, grayImage, CV_BGR2GRAY);
    cvtColor(image, hsvImage, CV_BGR2HSV);
    float meanHue = mean(hsvImage)[0];
    Moments m = moments(grayImage);
    HuMoments(m, featureVector.colRange(0, 6));
    featureVector.at<float>(1, 7) = meanHue / 360;
}

void train(CvSVM& classifier, Mat trainingFeatures, vector<int> classLabels, int kFolds)
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
        classes.at<float>(i, 1) = classLabels[i];
    }

    /* Now train the shit out of it */
    classifier.train_auto(trainingFeatures, classes, Mat(), Mat(), params, kFolds);
}

int classify(CvSVM& classifier, Mat image)
{
    /* Extract features */
    Mat featureVector(FEAT_SIZE, 1, CV_32FC1);
    feature(image, featureVector);

    /* Classify */
    float classLabel = classifier.predict(featureVector, true);

    /* Integer class label */
    return (int)classLabel;
}
