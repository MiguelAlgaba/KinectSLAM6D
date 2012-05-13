/*!
 *  \author    Miguel Algaba
 *  \date      2011-2012
 */

#define ENABLE_OPENNI_GRABBER  1
#define ENABLE_RAWLOG_GRABBER   0
#define ENABLE_GPU_SURF   0
#define ENABLE_ORB  1
#define ENABLE_STANFORD_GICP  1

#if ENABLE_RAWLOG_GRABBER
    #include "include/KinectGrabber_Rawlog.h"
#elif ENABLE_OPENNI_GRABBER
    #include "include/KinectGrabber_OpenNI.h"
#else
    #include "include/KinectGrabber_MRPT.h"
#endif

#include "include/FrameRGBD.h"

#if ENABLE_GPU_SURF
    #include "include/VisualFeatureDescriptorExtractor_SURF_GPU.h"
#elif ENABLE_ORB
    #include "include/VisualFeatureDescriptorExtractor_ORB.h"
#else
    #include "include/VisualFeatureDescriptorExtractor_Generic.h"
#endif

#include "include/VisualFeatureMatcher_Generic.h"

#include "include/Visual3DRigidTransformationEstimator_RANSAC.h"

#if ENABLE_STANFORD_GICP
    #include "include/ICPPoseRefiner_StanfordGICP.h"
#else
    #include "include/ICPPoseRefiner_PCL.h"
#endif

#include "include/Miscellaneous.h"

#include <pcl/visualization/cloud_viewer.h>

#include <pcl/io/pcd_io.h> //Save global map as PCD file

const int LOOP_DETECTION_THRESHOLD = 40;

//Video sequence
int main(int argc, char **argv)
{
    //Create the Kinect grabber objects
    #if ENABLE_RAWLOG_GRABBER
        KinectGrabber_Rawlog grabber = KinectGrabber_Rawlog(string(argv[1]));
    #elif ENABLE_OPENNI_GRABBER
        KinectGrabber_OpenNI grabber = KinectGrabber_OpenNI();
    #else
        KinectGrabber_MRPT grabber = KinectGrabber_MRPT();
    #endif

    FrameRGBD* frame1;
    cv::Mat imgKeypoints2D_1;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudKeypoints3D_1;
    std::vector<cv::KeyPoint> keypoints1;
    cv::Mat descriptors1;

    FrameRGBD* frame2;
    cv::Mat imgKeypoints2D_2;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr pointCloudKeypoints3D_2;
    std::vector<cv::KeyPoint> keypoints2;
    cv::Mat descriptors2;

    //Create the 2d visual feature detector and descriptor extractor
    #if ENABLE_GPU_SURF
        VisualFeatureDescriptorExtractor_SURF_GPU extractor = VisualFeatureDescriptorExtractor_SURF_GPU();
    #elif ENABLE_ORB
        VisualFeatureDescriptorExtractor_ORB extractor = VisualFeatureDescriptorExtractor_ORB();
    #else
        VisualFeatureDescriptorExtractor_Generic extractor =
            VisualFeatureDescriptorExtractor_Generic(cv::FeatureDetector::create("SURF"),cv::DescriptorExtractor::create("SURF"));
    #endif

    //Create the descriptor matcher
    #if ENABLE_ORB
        VisualFeatureMatcher_Generic matcher = VisualFeatureMatcher_Generic(cv::DescriptorMatcher::create( "BruteForce" ),"NoneFilter");
    #else
        VisualFeatureMatcher_Generic matcher = VisualFeatureMatcher_Generic(cv::DescriptorMatcher::create( "BruteForce" ),"CrossCheckFilter");
    #endif

    //Create the visual 3D rigid transformation estimator
    Visual3DRigidTransformationEstimator_RANSAC transformationEstimator = Visual3DRigidTransformationEstimator_RANSAC();

    //Create the ICP pose refiner
    #if ENABLE_STANFORD_GICP
        ICPPoseRefiner_StanfordGICP poseRefiner = ICPPoseRefiner_StanfordGICP();
    #else
        ICPPoseRefiner_PCL poseRefiner = ICPPoseRefiner_PCL();
    #endif

    pcl::visualization::CloudViewer viewer("Keypoints 3D");

    bool endProgram=false;
    while(!endProgram)
    {
        //Grab a RGB-D frame
        grabber.grab();
        frame1=new FrameRGBD();grabber.getCurrentFrameRGBD(*frame1);

        //Detect features and compute descriptors on the second image
        std::vector<float> descriptors_aux;
        extractor.setInputImage(frame1->intensityImage);
        extractor.detectKeypointsAndComputeDescriptors(keypoints1,descriptors1,descriptors_aux);

        //Draw 2D keypoints over the image
        cv::drawKeypoints(frame1->intensityImage,keypoints1,imgKeypoints2D_1,cv::Scalar(0,255,0));
        cv::Mat imgText1 = imgKeypoints2D_1.clone();
        cv::putText(imgText1,"Press enter to grab the first frame",cv::Point(30,450),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),2);
        cv::imshow("Keypoints 2D",imgText1);

        //Draw 3D keypoitns over the point cloud
        pointCloudKeypoints3D_1.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::copyPointCloud(*frame1->pointCloudPtr,*pointCloudKeypoints3D_1);
        for(int kpIndex=0;kpIndex<keypoints1.size();kpIndex++)
        {
            int y=keypoints1[kpIndex].pt.x;
            int x=keypoints1[kpIndex].pt.y;
            for(int rIndex=-2;rIndex<=2;rIndex++)
            {
                for(int cIndex=-2;cIndex<=2;cIndex++)
                {
                    int pointIndex=(x+cIndex)*640+(y+rIndex);
                    if(pointIndex>=0 && pointIndex<640*480)
                    {
                        pointCloudKeypoints3D_1->points[pointIndex].r=0;
                        pointCloudKeypoints3D_1->points[pointIndex].g=255;
                        pointCloudKeypoints3D_1->points[pointIndex].b=0;
                    }
                }
            }
        }
        viewer.showCloud(pointCloudKeypoints3D_1);

        //Check if the user has stopped the program
        endProgram = cv::waitKey(10)=='\n';
        if(!endProgram)
        {
            //Free memory
            delete frame1;
        }
    }

    endProgram=false;
    while(!endProgram)
    {
        //Grab a RGB-D frame
        grabber.grab();
        frame2=new FrameRGBD();grabber.getCurrentFrameRGBD(*frame2);

        //Detect features and compute descriptors on the second image
        std::vector<float> descriptors_aux;
        extractor.setInputImage(frame2->intensityImage);
        extractor.detectKeypointsAndComputeDescriptors(keypoints2,descriptors2,descriptors_aux);

        //Draw 2D keypoints over the image
        cv::drawKeypoints(frame2->intensityImage,keypoints2,imgKeypoints2D_2,cv::Scalar(0,255,0));
        cv::Mat imgText2 = imgKeypoints2D_2.clone();
        cv::putText(imgText2,"Press enter to grab the second frame",cv::Point(10,450),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),2);
        cv::imshow("Keypoints 2D",imgText2);

        //Draw 3D keypoitns over the point cloud
        pointCloudKeypoints3D_2.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
        pcl::copyPointCloud(*frame2->pointCloudPtr,*pointCloudKeypoints3D_2);
        for(int kpIndex=0;kpIndex<keypoints2.size();kpIndex++)
        {
            int y=keypoints2[kpIndex].pt.x;
            int x=keypoints2[kpIndex].pt.y;
            for(int rIndex=-2;rIndex<=2;rIndex++)
            {
                for(int cIndex=-2;cIndex<=2;cIndex++)
                {
                    int pointIndex=(x+cIndex)*640+(y+rIndex);
                    if(pointIndex>=0 && pointIndex<640*480)
                    {
                        pointCloudKeypoints3D_2->points[pointIndex].r=0;
                        pointCloudKeypoints3D_2->points[pointIndex].g=255;
                        pointCloudKeypoints3D_2->points[pointIndex].b=0;
                    }
                }
            }
        }
        viewer.showCloud(pointCloudKeypoints3D_2);

        //Check if the user has stopped the program
        endProgram = cv::waitKey(10)=='\n';
        if(!endProgram)
        {
            //Free memory
            delete frame2;
        }
    }

    //Stop grabbing RGBD-frames
    grabber.stopGrabber();

    //Estimate the pose aproximation using feature matching
    std::vector<cv::DMatch> matches;
    matcher.match(descriptors1,descriptors2,matches);

    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    std::vector<char> matchesMask;
    static int numberInliers;
    numberInliers = matcher.outlierRemovalFundamentalMat(keypoints1,keypoints2,matches,matchesMask,3.0);
    matcher.get2DMatchedPoints(keypoints1,keypoints2,matches,points1,points2,numberInliers,matchesMask);

    cv::Mat drawImg;
    cv::drawMatches(imgKeypoints2D_1, keypoints1,imgKeypoints2D_2, keypoints2,
                    matches, drawImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255) ,matchesMask);

    //Estimate the 3D rigid transformation between frame 1 and 2
    static Eigen::Matrix4f H;
    transformationEstimator.estimateVisual3DRigidTransformation(points1,points2,frame1->pointCloudPtr,frame2->pointCloudPtr,H);

    //Perform ICP to refine the pose approximation
    poseRefiner.refinePose(*frame1,*frame2,H);

    //Save the image and point cloud with the 2D and 3D keypoints
    cv::imwrite("../results/Keypoints2D_1.jpg",imgKeypoints2D_1);
    pcl::io::savePCDFile("../results/Keypoints3D_1.pcd",*pointCloudKeypoints3D_1);
    cv::imwrite("../results/Keypoints2D_2.jpg",imgKeypoints2D_2);
    pcl::io::savePCDFile("../results/Keypoints3D_2.pcd",*pointCloudKeypoints3D_2);

    //Save the image with the 2D correspondences
    cv::imwrite("../results/Correspondences.jpg",drawImg);

    //Save the non-registered concatenated point clouds
        //Change first point cloud color to red
        for(int i=1;i<frame1->pointCloudPtr->size();i++)
        {
            frame1->pointCloudPtr->points[i].r = 255;
            frame1->pointCloudPtr->points[i].g = 0;
            frame1->pointCloudPtr->points[i].b = 0;
        }
        //Change second point cloud color to blue
        for(int i=1;i<frame2->pointCloudPtr->size();i++)
        {
            frame2->pointCloudPtr->points[i].r = 0;
            frame2->pointCloudPtr->points[i].g = 0;
            frame2->pointCloudPtr->points[i].b = 255;
        }
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr non_registered_map;
    non_registered_map.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    *non_registered_map+=*frame1->pointCloudPtr;
    *non_registered_map+=*frame2->pointCloudPtr;
    pcl::io::savePCDFile("../results/Non_registered_pointclouds.pcd",*non_registered_map);

    //Save the registered concatenated point clouds
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr registered_map;
    registered_map.reset(new pcl::PointCloud<pcl::PointXYZRGBA>());
    pcl::PointCloud<pcl::PointXYZRGBA> alignedCloud;
    pcl::transformPointCloud(*frame1->pointCloudPtr,alignedCloud,H);
    *registered_map+=alignedCloud;
    *registered_map+=*frame2->pointCloudPtr;
    pcl::io::savePCDFile("../results/Registered_pointclouds.pcd",*registered_map);

    return 0;
}
