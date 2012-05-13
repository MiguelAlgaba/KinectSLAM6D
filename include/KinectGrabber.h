#define ENABLE_POINTCLOUD_DOWNSAMPLER 1
#define ENABLE_APPROXIMATE_VOXEL_GRID 0

#include <mrpt/system/datetime.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h> //for copyPointCloud
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#if ENABLE_POINTCLOUD_DOWNSAMPLER
    #include "PointCloudDownsampler.h"
#elif ENABLE_APPROXIMATE_VOXEL_GRID
    #include <pcl/filters/approximate_voxel_grid.h>
#else
    #include <pcl/filters/voxel_grid.h>
#endif

#include "FrameRGBD.h"

/*!Abstract class KinectGrabber that especifies the functionality of a generic RGBD grabber. This class especifies methods to grab a FrameRGBD and stop grabbing.*/
class KinectGrabber
{
protected:
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentColouredPointCloudPtr;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr currentDownsampledPointCloudPtr;
	cv::Mat currentIntensityImage;
    uint64_t currentTimeStamp;

    #if ENABLE_POINTCLOUD_DOWNSAMPLER
        PointCloudDownsampler grid;
    #elif ENABLE_APPROXIMATE_VOXEL_GRID
        pcl::ApproximateVoxelGrid<pcl::PointXYZRGBA> grid;
    #else
        pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
    #endif

    #if ENABLE_POINTCLOUD_DOWNSAMPLER
    void initPointCloudDownsampler()
    {
        grid = PointCloudDownsampler(8);
    }
    #else
    void initVoxelGrid()
    {
        grid.setLeafSize(0.05,0.05,0.05);
        grid.setFilterFieldName ("z");
        grid.setFilterLimits (0.0,5.0);
    }
    #endif

public:
	/*!Returns the current 3D coloured point cloud.*/
	virtual void getCurrentColouredPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr)=0;
	/*!Returns a downsampled version of the provided 3D point cloud.*/
	virtual void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr,
                                  pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& downsampledPointCloudPtr)=0;
	/*!Returns the current intensity image provided by the RGB camera.*/
	virtual void getCurrentIntensityImage(cv::Mat& intensityImage)=0;
 	/*!Retains the current intensity image and its corresponding 3D point cloud.*/
	virtual void grab()=0;
	/*!Stop grabing RGBD frames.*/
	virtual void stopGrabber()=0;
    /*!Returns a RGBD frame containing the intensity image and 3D point cloud. It also computes a downsampled version of the 3D point cloud and adds it to the returned RGBD frame.*/
    virtual void getCurrentFrameRGBD(FrameRGBD&)=0;
};

