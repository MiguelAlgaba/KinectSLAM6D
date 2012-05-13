#include "../include/VisualFeatureDescriptorExtractor_SURF_GPU.h"

VisualFeatureDescriptorExtractor_SURF_GPU::VisualFeatureDescriptorExtractor_SURF_GPU()
{

}

//VisualFeatureDescriptorExtractor_SURF_GPU::~VisualFeatureDescriptorExtractor_SURF_GPU(){}

void VisualFeatureDescriptorExtractor_SURF_GPU::detectKeypointsAndComputeDescriptors(std::vector<cv::KeyPoint>& keypoints,cv::Mat& descriptors,std::vector<float>& descriptors_aux)
{
    //Detect features and extract descriptors from the image
    imgMatGPU=cv::gpu::GpuMat(inputImage);
    surf(imgMatGPU, cv::gpu::GpuMat(), keypointsGPU, descriptorsGPU);

    //Download keypoints and descriptors from device memory to host memory
    surf.downloadKeypoints(keypointsGPU, keypoints);
    surf.downloadDescriptors(descriptorsGPU, descriptors_aux);
    descriptors = cv::Mat(descriptors_aux).reshape(0,descriptorsGPU.rows);

    //Crop the descriptor matrix from Nx128 to Nx64 to speed up the feature matching process
    descriptors = descriptors(cv::Rect(0,0,64,descriptors.rows)).clone();
}
