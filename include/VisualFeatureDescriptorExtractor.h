#ifndef VISUAL_FEATURE_DESCRIPTOR_EXTRACTOR
#define VISUAL_FEATURE_DESCRIPTOR_EXTRACTOR

#include "opencv2/features2d/features2d.hpp"

/*!Abstract class VisualFeatureDescriptorExtractor that especifies the functionality of a generic visual 2D feature detector
 and descriptor extractor.*/
class VisualFeatureDescriptorExtractor
{
protected:
    cv::Mat inputImage;
public:
	/*!Sets the input image to which the keypoints and descriptors will be computed.*/
	virtual void setInputImage(const cv::Mat& image)
	{
		inputImage=image;
	}

	/*!Performs feature detection and descriptor extraction over the image provided by the method setInputImage.*/
	virtual void detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux)=0;
};
#endif
