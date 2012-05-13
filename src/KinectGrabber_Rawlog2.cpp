#include "../include/KinectGrabber_Rawlog2.h"

#include <pcl/common/transforms.h>

#include <iostream>

KinectGrabber_Rawlog2::KinectGrabber_Rawlog2(const std::string &rawlog_file = std::string())
{
     #if ENABLE_POINTCLOUD_DOWNSAMPLER
        KinectGrabber::initPointCloudDownsampler();
     #else
        KinectGrabber::initVoxelGrid();
     #endif

    //Open the rawlog file
    dataset = new mrpt::utils::CFileGZInputStream();
    if (!dataset->open(rawlog_file))
        throw std::runtime_error("Couldn't open rawlog dataset file for input...");

    // Set external images directory:
    mrpt::utils::CImage::IMAGES_PATH_BASE = mrpt::slam::CRawlog::detectImagesDirectory(rawlog_file);

    //Create empty point cloud containers for the 3D point cloud and its downsampled version
    currentColouredPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    currentDownsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
}

//KinectGrabber_Rawlog2::~KinectGrabber_Rawlog2(){}

void KinectGrabber_Rawlog2::grab()
{
    static bool grabbed; grabbed=false;

    while(!grabbed)
    {
        //Read observations until we get a CObservation3DRangeScan
        mrpt::slam::CObservationPtr obs;
        do
        {
            try
            {
                *dataset >> obs;
            }
            catch (std::exception &e)
            {
                throw std::runtime_error( std::string("\nError reading from dataset file (EOF?):\n")+std::string(e.what()) );
            }
        } while (!IS_CLASS(obs,CObservation3DRangeScan));

        // We have one observation:
        currentObservationPtr = CObservation3DRangeScanPtr(obs);
        currentObservationPtr->load(); // *Important* This is needed to load the range image if stored as a separate file.

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

            grabbed=true;
        }
    }
}

void KinectGrabber_Rawlog2::getCurrentColouredPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr)
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

void KinectGrabber_Rawlog2::filterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr,
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

void KinectGrabber_Rawlog2::getCurrentIntensityImage(cv::Mat& intensityImage)
{
    cv::cvtColor(currentIntensityImage,intensityImage,CV_BGR2GRAY);
}

void KinectGrabber_Rawlog2::getCurrentFrameRGBD(FrameRGBD& frameRGBD)
{
    getCurrentIntensityImage(frameRGBD.intensityImage);
    getCurrentColouredPointCloudPtr(frameRGBD.pointCloudPtr);
    filterPointCloud(frameRGBD.pointCloudPtr,frameRGBD.downsampledPointCloudPtr);
    frameRGBD.timeStamp=currentTimeStamp;
}
