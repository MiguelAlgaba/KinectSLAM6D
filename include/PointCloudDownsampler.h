#ifndef POINTCLOUD_DOWNSAMPLER
#define POINTCLOUD_DOWNSAMPLER

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*!This class implements a fast and straightforward algorithm to downsample organized 3D point clouds. The algorithm takes the median point of a square region (downsamplingStep x downsamplingStep) of 3D points to mitigate the data noise.*/
class PointCloudDownsampler
{
private:
  int downsamplingStep;
public:
  /*!Constructor of an instance of PointCloudDownsampler given the downsamplingStep*/
  PointCloudDownsampler(const int = 8);

  /*!Sets the desired downsamplingStep to the given value*/
  void setDownsamplingStep(const int);

  /*!Downsamples the point cloud given by pointCloudPtr skiping the RGB information. The resulting downsampled point cloud is returned in downsampledPointCloudPtr.*/
  void downsamplePointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& downsampledPointCloudPtr);

  /*!Downsamples the point cloud given by pointCloudPtr. The color information will be computed by taking the median RGB value in the square region. The resulting downsampled point cloud is returned in downsampledPointCloudPtr.*/
  void downsamplePointCloudColor(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr&);
};
#endif
