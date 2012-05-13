#include "../include/VisualFeatureDescriptorExtractor_Generic.h"

VisualFeatureDescriptorExtractor_Generic::VisualFeatureDescriptorExtractor_Generic(cv::Ptr<cv::FeatureDetector> featDetector, cv::Ptr<cv::DescriptorExtractor> descExtractor)
{
	featureDetector=featDetector;
	descriptorExtractor=descExtractor;
}

//VisualFeatureDescriptorExtractor_Generic::~VisualFeatureDescriptorExtractor_Generic(){}

void VisualFeatureDescriptorExtractor_Generic::detectKeypoints(std::vector<cv::KeyPoint>& keypoints)
{
	featureDetector->detect(inputImage,keypoints);
}

void VisualFeatureDescriptorExtractor_Generic::computeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors)
{
	descriptorExtractor->compute(inputImage,keypoints,descriptors);
}

void VisualFeatureDescriptorExtractor_Generic::detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptor_aux/*=std::vector<float>()*/)
{
   detectKeypoints(keypoints);
   computeDescriptors(keypoints,descriptors);
}
