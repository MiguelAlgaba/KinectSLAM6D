#include "../include/KinectGrabber_Rawlog.h"

#include <pcl/common/transforms.h>

#include <iostream>


// ----------------------------------------------------
// The offline grabbing thread
// ----------------------------------------------------
void thread_grabbing(KinectGrabber_Rawlog::TThreadParam &p)
{
	try
	{
		typedef std::auto_ptr<mrpt::hwdrivers::CKinect> CKinectPtr;  // This assures automatic destruction

		// Only one of these will be actually used:
		CKinectPtr          kinect;
		mrpt::utils::CFileGZInputStream  dataset;

		mrpt::system::TTimeStamp  dataset_prev_tim     = INVALID_TIMESTAMP;
		mrpt::system::TTimeStamp  my_last_read_obs_tim = INVALID_TIMESTAMP;

        if (!dataset.open(p.rawlog_file))
            throw std::runtime_error("Couldn't open rawlog dataset file for input...");

        // Set external images directory:
        mrpt::utils::CImage::IMAGES_PATH_BASE = mrpt::slam::CRawlog::detectImagesDirectory(p.rawlog_file);

		mrpt::utils::CTicTac tictac;
		int nImgs = 0;

		while (!p.quit)
		{
            mrpt::slam::CObservationPtr obs;
            do
            {
                try {
                    dataset >> obs;
                }
                catch (std::exception &e) {
                    throw std::runtime_error( std::string("\nError reading from dataset file (EOF?):\n")+std::string(e.what()) );
                }
                ASSERT_(obs.present())
            } while (!IS_CLASS(obs,CObservation3DRangeScan));

            // We have one observation:
            CObservation3DRangeScanPtr obs3D = CObservation3DRangeScanPtr(obs);
            obs3D->load(); // *Important* This is needed to load the range image if stored as a separate file.

            // Do we have to wait to emulate real-time behavior?
            const mrpt::system::TTimeStamp cur_tim = obs3D->timestamp;
            const mrpt::system::TTimeStamp now_tim = mrpt::system::now();

            if (dataset_prev_tim!=INVALID_TIMESTAMP && my_last_read_obs_tim!=INVALID_TIMESTAMP)
            {
#ifndef FAKE_KINECT_FPS_RATE
                const double At_dataset = mrpt::system::timeDifference( dataset_prev_tim, cur_tim );
#else
                const double At_dataset = 1.0/FAKE_KINECT_FPS_RATE;
#endif
                const double At_actual  = mrpt::system::timeDifference( my_last_read_obs_tim, now_tim );

                const double need_to_wait_ms = 1000.*( At_dataset-At_actual );
                //cout << "[Kinect grab thread] Need to wait (ms): " << need_to_wait_ms << endl;
                if (need_to_wait_ms>0)
                    mrpt::system::sleep( need_to_wait_ms );
            }

            // Send observation to main thread:
            p.new_obs.set(obs3D);

            dataset_prev_tim     = cur_tim;
            my_last_read_obs_tim = mrpt::system::now();

			// Update Hz rate estimate
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
		std::cout << "Exception in Kinect thread: " << e.what() << std::endl;
		p.quit = true;
	}
}

KinectGrabber_Rawlog::KinectGrabber_Rawlog(const std::string &rawlog_file = std::string())
{
     #if ENABLE_POINTCLOUD_DOWNSAMPLER
        KinectGrabber::initPointCloudDownsampler();
     #else
        KinectGrabber::initVoxelGrid();
     #endif

    // Launch grabbing thread:
    thrParPtr=new KinectGrabber_Rawlog::TThreadParam(rawlog_file,false);

    thHandle = mrpt::system::createThreadRef(thread_grabbing ,*thrParPtr);

    while(!currentObservationPtr){currentObservationPtr = thrParPtr->new_obs.get();}

    currentColouredPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    currentDownsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
}

//KinectGrabber_Rawlog::~KinectGrabber_Rawlog(){}

void KinectGrabber_Rawlog::stopGrabber()
{
    std::cout << "Waiting for grabbing thread to exit...\n";
	thrParPtr->quit = true;
	mrpt::system::joinThread(thHandle);
}

void KinectGrabber_Rawlog::grab()
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
                currentObservationPtr->hasIntensityImage)
            {
                //Retain the point cloud of the current frame
                currentObservationPtr->project3DPointsFromDepthImageInto(*currentColouredPointCloudPtr, false /* without obs.sensorPose */);
                currentColouredPointCloudPtr->width=640*480;
                currentColouredPointCloudPtr->height=1;

                //Retain the RGB image of the current frame
                cv::Mat currentRGBImg_aux((IplImage *) currentObservationPtr->intensityImage.getAs<IplImage>());
                currentIntensityImage=currentRGBImg_aux;

                //Retain the timestamp of the current frame
                currentTimeStamp = currentObservationPtr->timestamp;

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

void KinectGrabber_Rawlog::getCurrentColouredPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr)
{
    colouredPointCloudPtr=currentColouredPointCloudPtr;

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

void KinectGrabber_Rawlog::filterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr,
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

void KinectGrabber_Rawlog::getCurrentIntensityImage(cv::Mat& intensityImage)
{
    cv::cvtColor(currentIntensityImage,intensityImage,CV_BGR2GRAY);
}

void KinectGrabber_Rawlog::getCurrentFrameRGBD(FrameRGBD& frameRGBD)
{
    getCurrentIntensityImage(frameRGBD.intensityImage);
    getCurrentColouredPointCloudPtr(frameRGBD.pointCloudPtr);
    filterPointCloud(frameRGBD.pointCloudPtr,frameRGBD.downsampledPointCloudPtr);
    frameRGBD.timeStamp=currentTimeStamp;
}
