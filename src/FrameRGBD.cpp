#include "../include/FrameRGBD.h"

FrameRGBD::FrameRGBD(){}

FrameRGBD::~FrameRGBD(){}

void copyGICPPointSetFromPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pclpointcloudptr,dgc::gicp::GICPPointSet& gicppointset)
{
    for(int i=0;i<pclpointcloudptr->size();i++)
    {
        dgc::gicp::GICPPoint point;
        point.x=pclpointcloudptr->points[i].x;
        point.y=pclpointcloudptr->points[i].y;
        point.z=pclpointcloudptr->points[i].z;
        gicppointset.AppendPoint(point);
    }
}

//Compute normal and covariance
void FrameRGBD::computeGICPNormalMatrices(const double gicp_epsilon)
{
    gicpPointSet = dgc::gicp::GICPPointSet();

    //Copy the downsampled point cloud to the GICP point cloud structure
    copyGICPPointSetFromPointCloudPtr(downsampledPointCloudPtr,gicpPointSet);

    // build kdtrees and normal matrices
    gicpPointSet.SetGICPEpsilon(gicp_epsilon);
    gicpPointSet.BuildKDTree();
    gicpPointSet.ComputeMatrices();
}
