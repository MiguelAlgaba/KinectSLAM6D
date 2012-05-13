#ifndef FRAME_RGBD
#define FRAME_RGBD

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "gicp.h"

/*!
The class FrameRGBD encapsulates the RGB and 3D data of a certain frame. It contains the intensity image provided by the RGB camera, the 3D coloured point cloud reconstructed from the depth and RGB images. It also contains a downsampled version of the 3D point cloud.
*/
class FrameRGBD
{
public:
  /*!Pointer to the coloured 3D point cloud*/
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudPtr;
  /*!Pointer to the downsampled version of the point cloud*/
  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr downsampledPointCloudPtr;
  /*!Grayscale intensity image*/
  cv::Mat intensityImage;
  /*!Downsampled point cloud with normals for the GICP algorithm*/
  dgc::gicp::GICPPointSet gicpPointSet;
  uint64_t timeStamp;
  FrameRGBD();
  ~FrameRGBD();

  /*!
  Compute the point cloud normals and covariance matrices for GICP
  */
  void computeGICPNormalMatrices(const double = 1e-3);
};
#endif
