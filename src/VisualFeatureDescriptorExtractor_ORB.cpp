#include "../include/VisualFeatureDescriptorExtractor_ORB.h"

VisualFeatureDescriptorExtractor_ORB::VisualFeatureDescriptorExtractor_ORB()
{
    //Create a ORF feature detector and descriptor extractor to detect N=256 features
    orb=cv::ORB(256);
}

//VisualFeatureDescriptorExtractor_ORB::~VisualFeatureDescriptorExtractor_ORB(){}

void VisualFeatureDescriptorExtractor_ORB::detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux)
{
    //Detect features and extract descriptors from the image
    orb(inputImage,cv::Mat(),keypoints,descriptors);
}
