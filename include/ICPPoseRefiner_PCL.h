#include <pcl/registration/icp.h>

#include "ICPPoseRefiner.h"

/*!This class encapsulates the functionality of a 3D rigid transformation refiner using the standard ICP algorithm implemented in the PCL library.*/
class ICPPoseRefiner_PCL : public ICPPoseRefiner
{
private:
    pcl::IterativeClosestPoint<pcl::PointXYZRGBA, pcl::PointXYZRGBA> icp;
    pcl::PointCloud<pcl::PointXYZRGBA> alignedCloudPtr;
    float maxCorrespondenceDistance;
    int maximumIterations;
    float transformationEpsilon;
public:
  ICPPoseRefiner_PCL();
  /*!This method refines the 3D rigid transformation between frame1 and frame2 giving H as an initial guess. The refined rigid transformation will be returned in H.*/
  void refinePose(FrameRGBD& frame1,
                  FrameRGBD& frame2,
                  Eigen::Matrix4f& H);
};
