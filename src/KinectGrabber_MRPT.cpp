#include "../include/KinectGrabber_MRPT.h"

#include <pcl/common/transforms.h>

#include <iostream>

// ----------------------------------------------------
// The offline grabbing thread
// ----------------------------------------------------
void thread_grabbing(KinectGrabber_MRPT::TThreadParam &p)
{
	try
	{
		mrpt::hwdrivers::CKinect  kinect;

		// Open:
		cout << "Calling CKinect::initialize()...";
		kinect.initialize();
		cout << "OK\n";

		CTicTac tictac;
		int nImgs = 0;
		bool there_is_obs=true, hard_error=false;

		while (!hard_error && !p.quit)
		{
			// Grab new observation from the camera:
			CObservation3DRangeScanPtr  obs     = CObservation3DRangeScan::Create(); // Smart pointers to observations
			CObservationIMUPtr          obs_imu = CObservationIMU::Create();

			kinect.getNextObservation(*obs,*obs_imu,there_is_obs,hard_error);

			if (!hard_error && there_is_obs)
			{
				p.new_obs.set(obs);
				p.new_obs_imu.set(obs_imu);
			}

			nImgs++;
			if (nImgs>10)
			{
				p.Hz = nImgs / tictac.Tac();
				nImgs=0;
				tictac.Tic();
			}
		}
	}
	catch(std::exception &e)
	{
		cout << "Exception in Kinect thread: " << e.what() << endl;
		p.quit = true;
	}
}

KinectGrabber_MRPT::KinectGrabber_MRPT()
{
     #if ENABLE_POINTCLOUD_DOWNSAMPLER
        KinectGrabber::initPointCloudDownsampler();
     #else
        KinectGrabber::initVoxelGrid();
     #endif

    // Launch grabbing thread:
    thrParPtr=new KinectGrabber_MRPT::TThreadParam();

    thHandle = mrpt::system::createThreadRef(thread_grabbing ,*thrParPtr);

    while(!currentObservationPtr){currentObservationPtr = thrParPtr->new_obs.get();}

    currentColouredPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    currentDownsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());

}

//KinectGrabber_MRPT::~KinectGrabber_MRPT(){}

void KinectGrabber_MRPT::stopGrabber()
{
    cout << "Waiting for grabbing thread to exit...\n";
	thrParPtr->quit = true;
	mrpt::system::joinThread(thHandle);
}

void KinectGrabber_MRPT::grab()
{
    static mrpt::system::TTimeStamp  last_obs_tim = INVALID_TIMESTAMP;
    static bool grabbed; grabbed=false;
    static const int imageSize = 640*480;
    static const int imageCenterIndex = 240*640+320;

    while(!grabbed)
    {
    	currentObservationPtr = thrParPtr->new_obs.get();

        //Check if the observation has a valid timestamp
	    if (currentObservationPtr && currentObservationPtr->timestamp!=INVALID_TIMESTAMP && currentObservationPtr->timestamp!=last_obs_tim )
        {
            // It is a new observation:
            last_obs_tim = currentObservationPtr->timestamp;

            if (currentObservationPtr->hasRangeImage &&
                currentObservationPtr->hasIntensityImage &&
                currentObservationPtr->points3D_x.size()==imageSize)
            {

                //Grab the RGB image
                cv::Mat currentRGBImg_aux((IplImage *) currentObservationPtr->intensityImage.getAs<IplImage>());
                currentIntensityImage=currentRGBImg_aux;

                //Retain the timestamp of the current frame
                currentTimeStamp = currentObservationPtr->timestamp;

                //Grab the coloured point cloud
                currentObservationPtr->project3DPointsFromDepthImageInto(*currentColouredPointCloudPtr, false /* without obs.sensorPose */);
                currentColouredPointCloudPtr->width=640*480;
                currentColouredPointCloudPtr->height=1;

                //Check if the grabbed frame has valid data
                if(currentColouredPointCloudPtr->size()==imageSize &&
                   currentColouredPointCloudPtr->points[imageCenterIndex].x!=0 &&
                   currentColouredPointCloudPtr->points[imageCenterIndex].y!=0 &&
                   currentColouredPointCloudPtr->points[imageCenterIndex].z!=0 &&
                   currentIntensityImage.rows*currentIntensityImage.cols==imageSize)
                {
                   grabbed=true;
                }
            }
        }
    }

}

void KinectGrabber_MRPT::getCurrentColouredPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr)
{
    //Change MRPT coordinate system to the PCL system
    Eigen::Matrix4f rotationMatrix;
    rotationMatrix(0,0)=0;
    rotationMatrix(0,1)=-1;
    rotationMatrix(0,2)=0;
    rotationMatrix(0,3)=0;
    rotationMatrix(1,0)=0;
    rotationMatrix(1,1)=0;
    rotationMatrix(1,2)=-1;
    rotationMatrix(1,3)=0;
    rotationMatrix(2,0)=1;
    rotationMatrix(2,1)=0;
    rotationMatrix(2,2)=0;
    rotationMatrix(2,3)=0;
    rotationMatrix(3,0)=0;
    rotationMatrix(3,1)=0;
    rotationMatrix(3,2)=0;
    rotationMatrix(3,3)=1;

    colouredPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::transformPointCloud(*currentColouredPointCloudPtr,*colouredPointCloudPtr,rotationMatrix);
}

void KinectGrabber_MRPT::filterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr,
                                            pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& downsampledPointCloudPtr)
{
    #if ENABLE_POINTCLOUD_DOWNSAMPLER
        grid.downsamplePointCloud(colouredPointCloudPtr,downsampledPointCloudPtr);
    #else
        grid.setInputCloud (colouredPointCloudPtr);
        currentDownsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA> );
        grid.filter (*currentDownsampledPointCloudPtr);
        downsampledPointCloudPtr = currentDownsampledPointCloudPtr;
    #endif
}

void KinectGrabber_MRPT::getCurrentIntensityImage(cv::Mat& intensityImage)
{
    //Get a grayscale version of the RGB image
    cv::cvtColor(currentIntensityImage,intensityImage,CV_BGR2GRAY);
}

void KinectGrabber_MRPT::getCurrentFrameRGBD(FrameRGBD& frameRGBD)
{
    getCurrentIntensityImage(frameRGBD.intensityImage);
    getCurrentColouredPointCloudPtr(frameRGBD.pointCloudPtr);
    filterPointCloud(frameRGBD.pointCloudPtr,frameRGBD.downsampledPointCloudPtr);
    frameRGBD.timeStamp=currentTimeStamp;
}

