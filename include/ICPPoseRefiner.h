#ifndef ICP_POSE_REFINER
#define ICP_POSE_REFINER

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "FrameRGBD.h"

/*!Abstract class that defines a method to refine the rigid transformation between two RGBD frames using an ICP approach.*/
class ICPPoseRefiner
{

public:
    /*!This method refines the 3D rigid transformation between frame1 and frame2 giving H as an initial guess. The refined rigid transformation will be returned in H.*/
    virtual void refinePose(FrameRGBD& frame1,
                            FrameRGBD& frame2,
                            Eigen::Matrix4f& H)=0;
};

#endif
