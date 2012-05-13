#include "KinectGrabber.h"

#include <pcl/io/openni_grabber.h>
#include <pcl/io/openni_camera/openni_driver.h>

/*!This class captures RGBD frames from the Kinect sensor using the OpenNI interface provided by the PCL library to access the sensor data. It grabs the intensity image as well as its corresponding 3D point cloud.*/
class KinectGrabber_OpenNI : public KinectGrabber
{
private:
  cv::Mat currentRGBImg;
  cv::Mat currentBGRImg;
  pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr pointCloudPtr_aux;
  pcl::OpenNIGrabber* interface;

  void cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud);
  void image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& img);

public:
  KinectGrabber_OpenNI();

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
  void stopGrabber();

  /*!Returns a RGBD frame containing the intensity image and 3D point cloud. It also computes a downsampled version of the 3D point cloud and adds it to the returned RGBD frame.*/
  void getCurrentFrameRGBD(FrameRGBD&);

};
