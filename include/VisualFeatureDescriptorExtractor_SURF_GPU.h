#include "VisualFeatureDescriptorExtractor.h"
#include "opencv2/gpu/gpu.hpp"

/*!This class encapsulates the functionality of a SURF 2D visual feature detector and descriptor extractor using the OpenCV library and CUDA. This class integrates a GPU accelerated implementation of SURF.*/
class VisualFeatureDescriptorExtractor_SURF_GPU : public VisualFeatureDescriptorExtractor
{
private:
	cv::gpu::SURF_GPU surf;
	std::vector<float> descriptors_aux;
	cv::gpu::GpuMat imgMatGPU;
    cv::gpu::GpuMat keypointsGPU;
    cv::gpu::GpuMat descriptorsGPU;
public:
  	VisualFeatureDescriptorExtractor_SURF_GPU();

    /*!Performs SURF feature detection and descriptor extraction over the image provided by the method setInputImage.*/
  	void detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux);
};
