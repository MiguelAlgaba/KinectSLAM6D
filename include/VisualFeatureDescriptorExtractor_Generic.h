#include "VisualFeatureDescriptorExtractor.h"

/*!This class encapsulates the functionality of a generic 2D visual feature detector and descriptor extractor using the OpenCV library.*/
class VisualFeatureDescriptorExtractor_Generic : public VisualFeatureDescriptorExtractor
{
private:
	cv::Ptr<cv::FeatureDetector> featureDetector;
	cv::Ptr<cv::DescriptorExtractor> descriptorExtractor;
public:
	/*!Constructor of an instance of VisualFeatureDescriptorExtractor_Generic given a feature detector and descriptor extractor.*/
  	VisualFeatureDescriptorExtractor_Generic(cv::Ptr<cv::FeatureDetector> featDetector,cv::Ptr<cv::DescriptorExtractor> descExtractor);

	/*!Performs feature detection and descriptor extraction over the image provided by the method setInputImage.*/  	
	void detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux);

	/*!Performs feature detection over the image provided by the method setInputImage.*/
	void detectKeypoints(std::vector<cv::KeyPoint>& keypoints);

	/*!Performs descriptor extraction over the image provided by the method setIntputImage.*/
	void computeDescriptors(std::vector<cv::KeyPoint>& descriptors_aux,cv::Mat& descriptors);
};
