#include "../include/KinectGrabber_OpenNI.h"

boost::mutex mtx_;

void KinectGrabber_OpenNI::cloud_cb_ (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr &cloud)
{
    //lock while we set our cloud;
    boost::mutex::scoped_lock lock (mtx_);
    pointCloudPtr_aux = cloud;
}

void KinectGrabber_OpenNI::image_cb_ (const boost::shared_ptr<openni_wrapper::Image>& img)
{
    img->fillRGB(currentBGRImg.cols,currentBGRImg.rows,currentBGRImg.data,currentBGRImg.step);
}

KinectGrabber_OpenNI::KinectGrabber_OpenNI()
{
     currentRGBImg = cv::Mat(480,640,CV_8UC3);
     currentBGRImg = cv::Mat(480,640,CV_8UC3);

     #if ENABLE_POINTCLOUD_DOWNSAMPLER
        KinectGrabber::initPointCloudDownsampler();
     #else
        KinectGrabber::initVoxelGrid();
     #endif

     interface = new pcl::OpenNIGrabber();

     // make callback functions from member functions
     boost::function<void
     (const pcl::PointCloud<pcl::PointXYZRGBA>::ConstPtr&)> f = boost::bind (&KinectGrabber_OpenNI::cloud_cb_, this, _1);

     boost::function<void
     (const boost::shared_ptr<openni_wrapper::Image>&)> g = boost::bind (&KinectGrabber_OpenNI::image_cb_, this, _1);

     // connect callback function for desired signal.
     interface->registerCallback (g);
     interface->registerCallback (f);

     // start receiving point clouds
     interface->start ();

     //currentColouredPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
     currentDownsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());

     sleep(2);

}

//KinectGrabber_OpenNI::~KinectGrabber_OpenNI(){}

void KinectGrabber_OpenNI::stopGrabber()
{
    interface->stop();
}

void KinectGrabber_OpenNI::grab()
{
    //lock while we set our cloud;
    boost::mutex::scoped_lock lock (mtx_);

    //Grab the current point cloud
    currentColouredPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::copyPointCloud(*pointCloudPtr_aux,*currentColouredPointCloudPtr);

    //Grab the current RGB image
    cv::cvtColor(currentBGRImg,currentIntensityImage,CV_RGB2BGR);

    //Retain the timestamp of the current frame
    currentTimeStamp = mrpt::system::getCurrentTime();
}

void KinectGrabber_OpenNI::getCurrentColouredPointCloudPtr(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr)
{
    colouredPointCloudPtr=currentColouredPointCloudPtr;
}

void KinectGrabber_OpenNI::filterPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& colouredPointCloudPtr,
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

void KinectGrabber_OpenNI::getCurrentIntensityImage(cv::Mat& intensityImage)
{
    //Get a grayscale version of the RGB image
    cv::cvtColor(currentIntensityImage,intensityImage,CV_BGR2GRAY);
}

void KinectGrabber_OpenNI::getCurrentFrameRGBD(FrameRGBD& frameRGBD)
{
    getCurrentIntensityImage(frameRGBD.intensityImage);
    getCurrentColouredPointCloudPtr(frameRGBD.pointCloudPtr);
    filterPointCloud(frameRGBD.pointCloudPtr,frameRGBD.downsampledPointCloudPtr);
    frameRGBD.timeStamp=currentTimeStamp;
}
