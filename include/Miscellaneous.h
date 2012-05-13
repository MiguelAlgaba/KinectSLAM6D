#ifndef MISCELLANEOUS
#define MISCELLANEOUS

#include <Eigen/Core>
#include <Eigen/Dense>
#include <iostream>
#include <fstream>
#include <vector>
#include "FrameRGBD.h"
#include <pcl/common/transforms.h>

#include <pcl/filters/voxel_grid.h>

#include "PointCloudDownsampler.h"


namespace Miscellaneous
{

    double rad2deg(const double angle);

	bool poseHasChanged(Eigen::Matrix4f& H1,Eigen::Matrix4f& H2,
		            double traslationThres,double rotationThres);

    void printRelativePosesAndRelativeLoopPoses
      (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&,
       std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&,
       std::vector<int>&,
       int);

    void printAccumulatedPosesAndAccumulatedLoopPoses
      (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&,
       std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&);

    void generateGlobalMapPtr(std::vector<FrameRGBD*>&,
                              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&,
                              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);

    void saveMatrix(Eigen::Matrix4f&,const char*);

    void saveTrajectory(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >&,
                        std::vector<FrameRGBD*>&,
                        const char*);

}

#endif
