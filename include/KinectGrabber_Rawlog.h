#include <mrpt/hwdrivers.h>
#include <mrpt/maps.h>
#include <mrpt/slam/CRawlog.h>
#include <memory> // for std::auto_ptr<>
#include <mrpt/slam/PCL_adapters.h>

#include "KinectGrabber.h"

using namespace mrpt::slam;

/*!This class captures RGBD frames from rawlog sensor data using the MRPT library. It grabs the intensity image as well as its corresponding 3D point cloud. It also grabs the timestamp of each frame if provided in the rawlog.*/
class KinectGrabber_Rawlog : public KinectGrabber
{

//  Uncoment to force the simulated sensor to run at this given rate.
//  If commented out, the simulated sensor will reproduce the real rate
//  as indicated by timestamps in the dataset.
#define FAKE_KINECT_FPS_RATE /*1*//*2*//*5*//*3*/30   // In Hz

private:
  mrpt::slam::CObservation3DRangeScanPtr currentObservationPtr;
  mrpt::system::TThreadHandle thHandle;

public:
  /*!Creates a KinectGrabber_Rawlog instance that grabs RGBD frames from the specified rawlog file.*/
  KinectGrabber_Rawlog(const std::string &rawlog_file);

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

// Thread for offline capturing: This should be done in another thread
struct TThreadParam
{
	TThreadParam(
		const std::string &_rawlog_file = std::string(),
		bool _generate_3D_pointcloud_in_this_thread = false)
		:	rawlog_file(_rawlog_file),
			generate_3D_pointcloud_in_this_thread(_generate_3D_pointcloud_in_this_thread),
			quit(false),
			Hz(0)
	{ }

	const std::string   rawlog_file; //!< Only when is_online==false
	const bool generate_3D_pointcloud_in_this_thread; //!< true: populate the 3D point fields in the output observation; false: only RGB and Depth images.

	volatile bool   quit;       //!< In/Out variable: Forces the thread to exit or indicates an error in the thread that caused it to end.
	volatile double Hz;         //!< Out variable: Approx. capturing rate from the thread.

	mrpt::synch::CThreadSafeVariable<mrpt::slam::CObservation3DRangeScanPtr> new_obs;  //!< RGB+D (+ optionally, 3D point cloud)
};

  TThreadParam* thrParPtr;

};
