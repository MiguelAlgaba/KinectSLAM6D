#ifndef VISUAL_3D_RIGID_TRANSFORMATION_ESTIMATOR
#define VISUAL_3D_RIGID_TRANSFORMATION_ESTIMATOR

#include <vector>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/features2d/features2d.hpp"

/*!Abstract class that specifies a method to estimate the 3D rigid transformation that best align a pair of 3D correspondences of two given point clouds.*/
class Visual3DRigidTransformationEstimator
{
protected:

public:
	/*!Estimates the 3D rigid transformation that best align a pair of 3D point correspondences of two given point clouds. The method takes two vectors of 2D points that determines the pairs of correspondences and two 3D point clouds. The algorithm then takes the 3D points corresponding to the 2D correspondences to estimate the 3D rigid transformation.*/
	virtual int estimateVisual3DRigidTransformation(
            const std::vector<cv::Point2f>& points1,
            const std::vector<cv::Point2f>& points2,
            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr1,
            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr2,
            Eigen::Matrix4f& rigidTransformation)=0;
};
#endif



