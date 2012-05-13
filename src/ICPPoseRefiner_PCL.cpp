#include "../include/ICPPoseRefiner_PCL.h"

ICPPoseRefiner_PCL::ICPPoseRefiner_PCL()
{
    maxCorrespondenceDistance = 0.1;
    maximumIterations = 16;
    transformationEpsilon = 1e-5;
    // Set the max correspondence distance to Xcm (e.g., correspondences with higher distances will be ignored)
    icp.setMaxCorrespondenceDistance (maxCorrespondenceDistance);
    // Set the maximum number of iterations (criterion 1)
    icp.setMaximumIterations (maximumIterations);
    // Set the transformation epsilon (criterion 2)
    icp.setTransformationEpsilon (transformationEpsilon);

}

//ICPPoseRefiner_PCL::~ICPPoseRefiner_PCL(){}

void ICPPoseRefiner_PCL::refinePose(FrameRGBD& frame1,
                                    FrameRGBD& frame2,
                                    Eigen::Matrix4f& H)
{
    icp.setInputCloud(frame1.downsampledPointCloudPtr);
    icp.setInputTarget(frame2.downsampledPointCloudPtr);
    icp.align(alignedCloudPtr,H);
    H = icp.getFinalTransformation();
}
