#include "Visual3DRigidTransformationEstimator.h"
#include <pcl/registration/transformation_estimation_svd.h>

/*!This class encapsulates the functionality of a 3D rigid transformation estimator to compute a rigid transformation using the SVD algorithm. This implementation considers that there isn't incorrect correspondences and could produce incorrect rigid transformations in presence of outliers.*/
class Visual3DRigidTransformationEstimator_SVD : public Visual3DRigidTransformationEstimator
{
private:
    pcl::registration::TransformationEstimationSVD<pcl::PointXYZRGBA,pcl::PointXYZRGBA> transformationEstimator;
    pcl::Correspondences correspondences;
    double minDepthValue;
    double maxDepthValue;

    void matchesWith3DValidData(const std::vector<cv::Point2f>&,
                                const std::vector<cv::Point2f>&,
                           	    const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
				                const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

public:

    /*!Constructor of an instance of Visual3DRigidTransformationEstimator_SVD with a given minDepthValue and maxDepthValue that specifies a valid range of 3D points (in meters).*/
	Visual3DRigidTransformationEstimator_SVD(const double = 0.05,const double = 10.0);

    /*!Sets the minDepthValue to the specified value (in meters)*/
    void setMinDepthValue(const double);

    /*!Sets the maxDepthValue to the specified value (in meters)*/
    void setMaxDepthValue(const double);

    /*!Estimates the 3D rigid transformation that best align a pair of 3D point correspondences of two given point clouds. The method takes two vectors of 2D points that determines the pairs of correspondences and two 3D point clouds. The algorithm then takes the 3D points corresponding to the 2D correspondences to estimate the 3D rigid transformation. This method should not be used in presence of outliers. In that case use the robustified method encapsulated in the class Visual3DRigidTransformationEstimator_RANSAC.*/
	int estimateVisual3DRigidTransformation(
            const std::vector<cv::Point2f>& points1,
            const std::vector<cv::Point2f>& points2,
            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr1,
            const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr2,
            Eigen::Matrix4f& rigidTransformation);
};

