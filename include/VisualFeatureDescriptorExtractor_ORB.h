#include "VisualFeatureDescriptorExtractor.h"

/*!This class encapsulates the functionality of an ORB 2D visual feature detector and descriptor extractor using the OpenCV library.*/
class VisualFeatureDescriptorExtractor_ORB : public VisualFeatureDescriptorExtractor
{
private:
        cv::ORB orb;
public:
  	VisualFeatureDescriptorExtractor_ORB();

	/*!Performs ORB feature detection and descriptor extraction over the image provided by the method setInputImage.*/ 
  	void detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux);
};
