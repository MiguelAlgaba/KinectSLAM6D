/*!
 *  \author    Miguel Algaba
 *  \date      2011-2012
 */

#define ENABLE_OPENNI_GRABBER  0
#define ENABLE_RAWLOG_GRABBER   0  //Emulate a Kinect sensor from a rawlog dataset
#define ENABLE_RAWLOG_GRABBER2   1 //Reads every frame in the rawlog
#define ENABLE_GPU_SURF   0
#define ENABLE_ORB  1
#define ENABLE_HOMOGRAPHY_OUTLIER_REMOVAL 0
#define ENABLE_DRAW_MATCHES 1
#define ENABLE_RANSAC_RIGID_TRANSFORMATION_ESTIMATION  1
#define ENABLE_STANFORD_GICP 1
#define ENABLE_GRAPH_OPTIMIZATION_MRPT  0
#define ENABLE_GRAPH_OPTIMIZATION_G2O 1
#define ENABLE_ESTIMATE_2D_RGBD_ODOMETRY  1
#define ENABLE_KINECT_2D_SCAN_GRABBER   1

#include <mrpt/utils/CTimeLogger.h> //Profiling

#include <mrpt/slam/CRawlog.h> //Save the 2D odometry rawlog
#include <mrpt/slam/CObservationOdometry.h>

#if ENABLE_GRAPH_OPTIMIZATION_MRPT
    #include "include/GraphOptimizer_MRPT.h"
#endif

#if ENABLE_KINECT_2D_SCAN_GRABBER
    #include "include/CKinect2DRawlog.h"
#endif

#if ENABLE_RAWLOG_GRABBER
    #include "include/KinectGrabber_Rawlog.h"
#elif ENABLE_RAWLOG_GRABBER2
    #include "include/KinectGrabber_Rawlog2.h"
#elif ENABLE_OPENNI_GRABBER
    #include "include/KinectGrabber_OpenNI.h"
#else
    #include "include/KinectGrabber_MRPT.h"
#endif

#if ENABLE_GRAPH_OPTIMIZATION_G2O
    #include "include/GraphOptimizer_G2O.h"
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

#if ENABLE_RANSAC_RIGID_TRANSFORMATION_ESTIMATION
    #include "include/Visual3DRigidTransformationEstimator_RANSAC.h"
#else
    #include "include/Visual3DRigidTransformationEstimator_SVD.h"
#endif

#if ENABLE_STANFORD_GICP
    #include "include/ICPPoseRefiner_StanfordGICP.h"
#else
    #include "include/ICPPoseRefiner_PCL.h"
#endif

#include "include/Miscellaneous.h"

#include "include/KeyframeLoopDetector.h"

#include <iostream>
#include "opencv2/highgui/highgui.hpp"

#include <pcl/io/pcd_io.h> //Save global map as PCD file

const int LOOP_DETECTION_THRESHOLD = 40;

#if ENABLE_ORB
const int KEYFRAME_INLIERS_THRESHOLD = 70;
#else
const int KEYFRAME_INLIERS_THRESHOLD = 100;
#endif

#if ENABLE_ESTIMATE_2D_RGBD_ODOMETRY
void pose3DToPose2D(Eigen::Matrix4f & pose3D,mrpt::poses::CPose2D & pose2D)
{
    Eigen::Matrix4f transMat; //Transformation matrix from PCL to MRPT reference frame
    transMat(0,0)=0;    transMat(0,1)=0;     transMat(0,2)=1;    transMat(0,3)=0;
    transMat(1,0)=-1;    transMat(1,1)=0;      transMat(1,2)=0;   transMat(1,3)=0;
    transMat(2,0)=0;    transMat(2,1)=-1;      transMat(2,2)=0;    transMat(2,3)=0;
    transMat(3,0)=0;    transMat(3,1)=0;      transMat(3,2)=0;    transMat(3,3)=1;

    mrpt::math::CMatrixDouble44 pose3D_aux = transMat*pose3D;
    mrpt::poses::CPose3D pose3DMRPT = mrpt::poses::CPose3D(pose3D_aux);
    pose2D = mrpt::poses::CPose2D(pose3DMRPT);
    pose2D.phi(pose2D.phi()+M_PI/2);
}

void add2DOdometryToRawlog(Eigen::Matrix4f & pose,
                           FrameRGBD* frame,
                           mrpt::utils::CFileGZOutputStream& rawlogFile)
{
    mrpt::poses::CPose2D pose2D;
    pose3DToPose2D(pose,pose2D);

    //Save the 2D pose to the rawlog as an odometry observation
    mrpt::slam::CObservationOdometry obs;
    obs.odometry = pose2D;
    obs.timestamp = frame->timeStamp;
    obs.sensorLabel = "Kinect2DOdometry";
    rawlogFile << obs;
}

void add2DPoseToFile(Eigen::Matrix4f & pose3D,std::ofstream & posesFile)
{
    mrpt::poses::CPose2D pose2D;
    pose3DToPose2D(pose3D,pose2D);
    posesFile << pose2D.x() << "  " << pose2D.y() << "  " << pose2D.phi() << "\n";
}
#endif

//Video sequence
int main(int argc, char **argv)
{
    //Create the profiler for timing results
    mrpt::utils::CTimeLogger profiler(true);

    #if ENABLE_ESTIMATE_2D_RGBD_ODOMETRY
    //Create a file stream to save the 2D poses
    std::ofstream posesFile;
    posesFile.open("../results/poses.mat");

    //Create an object write the observations stream
	std::string fileName = std::string("../results/Rawlog_visualOdometry_Kinect2D.rawlog");
	mrpt::utils::CFileGZOutputStream rawlogFile(fileName);
    #endif

    //Create the Kinect grabber objects
    #if ENABLE_RAWLOG_GRABBER
        KinectGrabber_Rawlog grabber = KinectGrabber_Rawlog(string(argv[1]));
    #elif ENABLE_RAWLOG_GRABBER2
        KinectGrabber_Rawlog2 grabber = KinectGrabber_Rawlog2(string(argv[1]));
    #elif ENABLE_OPENNI_GRABBER
        KinectGrabber_OpenNI grabber = KinectGrabber_OpenNI();
    #else
        KinectGrabber_MRPT grabber = KinectGrabber_MRPT();
    #endif

    #if ENABLE_KINECT_2D_SCAN_GRABBER
        CKinect2DRawlog kinect(argv[1]);
        kinect.setMaximumRange(4);
    #endif

    FrameRGBD* currentFrame;
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMapPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
    #if ENABLE_GRAPH_OPTIMIZATION_MRPT || ENABLE_GRAPH_OPTIMIZATION_G2O
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr globalMapOptimizedPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
    #endif
    Eigen::Matrix4f pose = Eigen::Matrix4f::Identity ();
    Eigen::Matrix4f keyframeRelativePose = Eigen::Matrix4f::Identity();

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
    #if ENABLE_RANSAC_RIGID_TRANSFORMATION_ESTIMATION
        Visual3DRigidTransformationEstimator_RANSAC transformationEstimator = Visual3DRigidTransformationEstimator_RANSAC();
    #else
        Visual3DRigidTransformationEstimator_SVD transformationEstimator = Visual3DRigidTransformationEstimator_SVD();
    #endif

    //Create the ICP pose refiner
    #if ENABLE_STANFORD_GICP
        ICPPoseRefiner_StanfordGICP poseRefiner = ICPPoseRefiner_StanfordGICP();
    #else
        ICPPoseRefiner_PCL poseRefiner = ICPPoseRefiner_PCL();
    #endif

    //Grab the first image
    grabber.grab();
    currentFrame=new FrameRGBD();grabber.getCurrentFrameRGBD(*currentFrame);
    #if ENABLE_STANFORD_GICP
        currentFrame->computeGICPNormalMatrices();
    #endif

    #if ENABLE_KINECT_2D_SCAN_GRABBER
        //Get 2D range observation
        mrpt::slam::CObservation2DRangeScan obs;
        kinect.get2DScan(obs);
        obs.sensorPose = mrpt::utils::CPose3D(0,0,0);

        //Save the observation to a file stream
        rawlogFile << obs;
    #endif

    //Detect features and compute descriptors on the first image
    std::vector<cv::KeyPoint> currentKeypoints;
    cv::Mat currentDescriptors;
    std::vector<float> currentDescriptors_aux;extractor.setInputImage(currentFrame->intensityImage);
    extractor.detectKeypointsAndComputeDescriptors(currentKeypoints,currentDescriptors,currentDescriptors_aux);

    //Create the loop detector
    KeyframeLoopDetector loopDetector = KeyframeLoopDetector(LOOP_DETECTION_THRESHOLD);

    //Add the first frame as a Keyframe
    loopDetector.addKeyframe(currentFrame);
    loopDetector.addPose(pose);
    loopDetector.addKeypoints(currentKeypoints);
    loopDetector.addDescriptors(currentDescriptors);
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > relativePoses;
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > accPoses;
    std::vector<Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6 > > > informationMatrices;
    #if ENABLE_GRAPH_OPTIMIZATION_MRPT || ENABLE_GRAPH_OPTIMIZATION_G2O
    //Add the first non-optimized pose
    accPoses.push_back(pose);
    std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > > accOptimizedPoses;
        #if ENABLE_GRAPH_OPTIMIZATION_MRPT
        //Add the first pose to the graph
        GraphOptimizer_MRPT optimizer = GraphOptimizer_MRPT();
        optimizer.addVertex(pose);
        #elif ENABLE_GRAPH_OPTIMIZATION_G2O
        //Add the first pose to the graph
        GraphOptimizer_G2O optimizer = GraphOptimizer_G2O();
        optimizer.addVertex(pose);
        #endif
    #endif
    std::vector<int> fromIndexes;
    int toIndex;

    //Add the first frame to the global map
    globalMapPtr->header.frame_id = currentFrame->pointCloudPtr->header.frame_id;
    *globalMapPtr += *currentFrame->pointCloudPtr;

    double loopTime;
    cv::namedWindow("correspondences",0);
    try
    {
        while(cv::waitKey(10)!='\n')
        {
            profiler.enter("Main loop");

            //Grab the second image
            profiler.enter("RGB-D frame grabbing");
            grabber.grab();
            currentFrame=new FrameRGBD();grabber.getCurrentFrameRGBD(*currentFrame);
            #if ENABLE_STANFORD_GICP
                currentFrame->computeGICPNormalMatrices();
            #endif
            profiler.leave("RGB-D frame grabbing");

            #if ENABLE_KINECT_2D_SCAN_GRABBER
                //Get 2D range observation
                mrpt::slam::CObservation2DRangeScan obs;
                kinect.get2DScan(obs);
                obs.sensorPose = mrpt::utils::CPose3D(0,0,0);

                //Save the observation to a file stream
                rawlogFile << obs;
            #endif

            //Detect features and compute descriptors on the second image
            profiler.enter("Feature detection and extraction");
            std::vector<cv::KeyPoint> currentKeypoints;
            cv::Mat currentDescriptors;
            std::vector<float> currentDescriptors_aux;
            extractor.setInputImage(currentFrame->intensityImage);
            extractor.detectKeypointsAndComputeDescriptors(currentKeypoints,currentDescriptors,currentDescriptors_aux);
            profiler.leave("Feature detection and extraction");

            profiler.enter("Feature matching");
            std::vector<cv::DMatch> matches;
            matcher.match(loopDetector.descriptorsListPointer()->back(),currentDescriptors,matches);
            profiler.leave("Feature matching");

            std::vector<cv::Point2f> points1;
            std::vector<cv::Point2f> points2;
            #if ENABLE_HOMOGRAPHY_OUTLIER_REMOVAL
                profiler.enter("Homography matrix outlier removal");
                std::vector<char> matchesMask;
                static int numberInliers;
                numberInliers = matcher.outlierRemovalHomography(loopDetector.keypointsListPointer->back(),currentKeypoints,matches,matchesMask,3.0);
                profiler.leave("Homography matrix outlier removal");
                std::cout << "numberInliers: " << numberInliers << std::endl;
            #else
                profiler.enter("Fundamental matrix outlier removal");
                std::vector<char> matchesMask;
                static int numberInliers;
                numberInliers = matcher.outlierRemovalFundamentalMat(loopDetector.keypointsListPointer()->back(),currentKeypoints,matches,matchesMask,3.0);
                profiler.leave("Fundamental matrix outlier removal");
                std::cout << "Number 2D inliers: " << numberInliers << std::endl;
            #endif

            //Get robust 2D matched points
            matcher.get2DMatchedPoints(loopDetector.keypointsListPointer()->back(),currentKeypoints,matches,points1,points2,numberInliers,matchesMask);

            #if ENABLE_DRAW_MATCHES
                cv::Mat drawImg;
                cv::drawMatches(loopDetector.keyframesPointer()->back()->intensityImage, loopDetector.keypointsListPointer()->back(), currentFrame->intensityImage, currentKeypoints,
                                matches, drawImg, CV_RGB(0, 255, 0), CV_RGB(0, 0, 255),matchesMask);
                cv::putText(drawImg,"Press enter to stop grabbing frames",cv::Point(20,450),cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(255,0,0),2);
                cv::imshow( "correspondences", drawImg );
            #endif

            //Estimate the 3D rigid transformation between frame 1 and 2
            profiler.enter("Rigid transformation estimation");
            Eigen::Matrix4f H = Eigen::Matrix4f::Identity();
            transformationEstimator.estimateVisual3DRigidTransformation(points1,points2,loopDetector.keyframesPointer()->back()->pointCloudPtr,currentFrame->pointCloudPtr,H);
            profiler.leave("Rigid transformation estimation");

            //Perform ICP to refine the pose approximation
            profiler.enter("Rigid transformation refinement ICP/GICP");
            poseRefiner.refinePose(*loopDetector.keyframesPointer()->back(),*currentFrame,H); //H is the inverse of the relative pose
            profiler.leave("Rigid transformation refinement ICP/GICP");

            if(numberInliers<KEYFRAME_INLIERS_THRESHOLD)
            {
                //Update the current pose with the rigid 3D transformation estimated from the previous and
                //current keyframes
                keyframeRelativePose = H.inverse();
                pose = pose*keyframeRelativePose;

                //Detect loops
                profiler.enter("Loop detection");
                loopDetector.addKeyframe(currentFrame);
                loopDetector.addKeypoints(currentKeypoints);
                loopDetector.addDescriptors(currentDescriptors);
                loopDetector.accumulateRelativePose(keyframeRelativePose);
                loopDetector.detectLoop(matcher,transformationEstimator,
                                        poseRefiner,
                                        relativePoses,
                                        informationMatrices,
                                        fromIndexes,toIndex);
                loopDetector.getCurrentPose(pose); //Update the current pose with the current keyframe pose
                profiler.leave("Loop detection");

                profiler.enter("Graph optimization");
                //Accumulate non-optimized poses
                accPoses.push_back(pose);

                //Add the keyframe vertex to the graph
                std::cout<<"Added vertex: "<<optimizer.addVertex(pose)<<std::endl;
                //Add the keyframe edges to the graph
                for(int i=0;i<relativePoses.size();i++)
                {
                    optimizer.addEdge(fromIndexes[i],toIndex,relativePoses[i],informationMatrices[i]);
                }

                //Optimize the graph
                optimizer.optimizeGraph();
                //Set the optimized poses to the loop detector
                optimizer.getPoses(accOptimizedPoses);
                loopDetector.setPoses(accOptimizedPoses);
                //Update the optimized pose
                pose=accOptimizedPoses.back();
                profiler.leave("Graph optimization");

                #if ENABLE_ESTIMATE_2D_RGBD_ODOMETRY
                    //Add a 2D pose estimate to the rawlog
                    add2DOdometryToRawlog(pose,currentFrame,rawlogFile);
                    add2DPoseToFile(pose,posesFile);
                #endif

            }
            else
            {
                #if ENABLE_ESTIMATE_2D_RGBD_ODOMETRY
                    //Add a 2D pose estimate to the rawlog
                    Eigen::Matrix4f currentPose = pose*H.inverse();
                    add2DOdometryToRawlog(currentPose,currentFrame,rawlogFile);
                    add2DPoseToFile(currentPose,posesFile);
                #endif

                //Free the current frame
                delete currentFrame;
            }

            profiler.leave("Main loop");

        }
    }
    catch (std::exception &e)
    {

    }

    //Stop grabbing RGBD-frames
    grabber.stopGrabber();

    //Save profiling results
    profiler.saveToCSVFile("../results/profiling.txt");
    loopDetector.saveLoopProfiling("../results/profiling_loop.txt");

    //Generate the global odometry map from keyframes and keyframe poses
    Miscellaneous::generateGlobalMapPtr(*loopDetector.keyframesPointer(),accPoses,globalMapPtr);

    //Save the global map as a PCD file
    pcl::io::savePCDFile("../results/pcd_files/global-map_visual_odometry.pcd",*globalMapPtr);

    //Save the trajectory with the timestamp
    #if ENABLE_RAWLOG_GRABBER
    Miscellaneous::saveTrajectory(accPoses,*loopDetector.keyframesPointer,"../results/graphs/TrajectoryTimestamp.txt");
    #endif

    optimizer.getPoses(accOptimizedPoses);
    Miscellaneous::generateGlobalMapPtr(*loopDetector.keyframesPointer(),accOptimizedPoses,globalMapOptimizedPtr);

    //Save the optimized global map as a PCD file
    pcl::io::savePCDFile("../results/pcd_files/global-map_optimized.pcd",*globalMapOptimizedPtr);

    //Save the optimized trajectory with the timestamp
    #if ENABLE_RAWLOG_GRABBER
    Miscellaneous::saveTrajectory(accOptimizedPoses,*loopDetector.keyframesPointer,"../results/graphs/OptimizedTrajectoryTimestamp.txt");
    #endif

    //Save the graph to file
    #if ENABLE_GRAPH_OPTIMIZATION_MRPT
    optimizer.saveGraph("../results/graphs/graph.graph");
    #elif ENABLE_GRAPH_OPTIMIZATION_G2O
    optimizer.saveGraph("../results/graphs/graph.g2o");
    #endif

    //Save the 2D trajectory with the timestamp
    #if ENABLE_ESTIMATE_2D_RGBD_ODOMETRY || ENABLE_KINECT_2D_SCAN_GRABBER
        //Close the rawlog stream file
        rawlogFile.close();
        posesFile.close();
    #endif

    #if ENABLE_RAWLOG_GRABBER //The dataset must contain the ground-truth
    if(argc==3)
    {
        //python evaluate_ate.py --plot trajectory_odometry.png groundtruth.txt TrajectoryTimestamp.txt
        //Compute ATE and RPE from the ground-truth and estimated trajectories
        char* terminal_command = (char*) malloc(1000);
        std::sprintf(terminal_command,"python ../tools/rgbd_benchmark_tools/evaluate_ate.py --plot ../results/trajectory_odometry.pdf %s ../results/graphs/TrajectoryTimestamp.txt;",argv[2]);
        std::system(terminal_command);delete terminal_command;

        terminal_command = (char*) malloc(1000);
        std::sprintf(terminal_command,"python ../tools/rgbd_benchmark_tools/evaluate_ate.py --plot ../results/trajectory_optimized.pdf %s ../results/graphs/OptimizedTrajectoryTimestamp.txt;",argv[2]);
        std::system(terminal_command);delete terminal_command;

        terminal_command = (char*) malloc(1000);
        std::sprintf(terminal_command,"python ../tools/rgbd_benchmark_tools/evaluate_rpe.py %s ../results/graphs/TrajectoryTimestamp.txt;",argv[2]);
        std::system(terminal_command);delete terminal_command;

        terminal_command = (char*) malloc(1000);
        std::sprintf(terminal_command,"python ../tools/rgbd_benchmark_tools/evaluate_rpe.py %s ../results/graphs/OptimizedTrajectoryTimestamp.txt;",argv[2]);
        std::system(terminal_command);delete terminal_command;
    }
    #endif

    return 0;
}
