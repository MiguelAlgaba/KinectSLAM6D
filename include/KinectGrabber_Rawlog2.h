#include <mrpt/hwdrivers.h>
#include <mrpt/maps.h>
#include <mrpt/slam/CRawlog.h>
#include <memory> // for std::auto_ptr<>
#include <mrpt/slam/PCL_adapters.h>

#include "KinectGrabber.h"

using namespace mrpt::slam;

/*!This class captures RGBD frames from rawlog sensor data using the MRPT library. It grabs the intensity image as well as its corresponding 3D point cloud. It also grabs the timestamp of each frame if provided in the rawlog.*/
class KinectGrabber_Rawlog2 : public KinectGrabber
{

private:
  mrpt::slam::CObservation3DRangeScanPtr currentObservationPtr;

  mrpt::utils::CFileGZInputStream*  dataset; //File stream to the rawlog dataset

  bool endGrabbing; //Bool variable to indicate that no more observations can be read from the rawlog

public:
  /*!Creates a KinectGrabber_Rawlog2 instance that grabs RGBD frames from the specified rawlog file.*/
  KinectGrabber_Rawlog2(const std::string &rawlog_file);

  /*!Returns the current 3D coloured point cloud.*/
  void getCurrentColouredPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr);

  /*!Returns a downsampled version of the provided 3D point cloud.*/
  void filterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr,
                        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& downsampledPointCloudPtr);

  /*!Returns the current intensity image provided by the RGB camera.*/
  void getCurrentIntensityImage(cv::Mat& intensityImage);

  /*!Retains the current intensity image and its corresponding 3D point cloud.*/
  void grab();

  /*!Stop grabing RGBD frames.*/
  inline void stopGrabber(){};

  /*!Returns a RGBD frame containing the intensity image and 3D point cloud. It also computes a downsampled version of the 3D point cloud and adds it to the returned RGBD frame.*/
  void getCurrentFrameRGBD(FrameRGBD&);
};
