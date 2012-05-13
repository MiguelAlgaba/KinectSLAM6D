#include <mrpt/hwdrivers.h>
#include <mrpt/maps.h>
#include <mrpt/slam/PCL_adapters.h>

#include "KinectGrabber.h"

using namespace mrpt::slam;

/*!This class captures RGBD frames from the Kinect sensor using the MRPT library to access the sensor data. It grabs the intensity image as well as its corresponding 3D point cloud.*/
class KinectGrabber_MRPT : public KinectGrabber
{

private:
  CObservation3DRangeScanPtr currentObservationPtr;
  mrpt::system::TThreadHandle thHandle;

public:
  KinectGrabber_MRPT();

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

// Thread for grabbing: Do this is another thread so we divide rendering and grabbing
//   and exploit multicore CPUs.
struct TThreadParam
{
	TThreadParam() : quit(false), pushed_key(0), tilt_ang_deg(0), Hz(0) { }

	volatile bool   quit;
	volatile int    pushed_key;
	volatile double tilt_ang_deg;
	volatile double Hz;

	mrpt::synch::CThreadSafeVariable<CObservation3DRangeScanPtr> new_obs;     // RGB+D (+3D points)
	mrpt::synch::CThreadSafeVariable<CObservationIMUPtr>         new_obs_imu; // Accelerometers
};

  TThreadParam* thrParPtr;

};
