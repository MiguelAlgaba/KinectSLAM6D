#define ENABLE_HOMOGRAPHY_OUTLIER_REMOVAL 0
#define ENABLE_RIGID_TRANSFORMATION_REFINEMENT 1
#define ENABLE_DEBUG_KEYFRAME_ALIGNMENT  0  //Don't enable if not necessary
#if ENABLE_DEBUG_KEYFRAME_ALIGNMENT
    #include <mrpt/base.h>
    #include <pcl/io/pcd_io.h>
    #include <pcl/filters/voxel_grid.h>
#endif

#include "../include/KeyframeLoopDetector.h"

KeyframeLoopDetector::KeyframeLoopDetector(const int numInliersThres)
{
    numberInliersThreshold=numInliersThres;

    //Enable the profiler
    profiler.enable(true);
}

void KeyframeLoopDetector::addKeyframe(FrameRGBD* keyframe)
{
    keyframes.push_back(keyframe);
}

void KeyframeLoopDetector::addPose(Eigen::Matrix4f& pose)
{
    poses.push_back(pose);
}

void KeyframeLoopDetector::addKeypoints(std::vector<cv::KeyPoint>& keypoints)
{
    keypointsList.push_back(keypoints);
}

void KeyframeLoopDetector::addDescriptors(cv::Mat& descriptors)
{
    descriptorsList.push_back(descriptors.clone());
}

void KeyframeLoopDetector::detectLoop(VisualFeatureMatcher_Generic& matcher,
                                      Visual3DRigidTransformationEstimator& transformationEstimator,
                                      ICPPoseRefiner& poseRefiner,
                                      std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& relativePoses,
                                      std::vector<Eigen::Matrix<double,6,6>, Eigen::aligned_allocator<Eigen::Matrix<double,6,6 > > >& informationMatrices,
                                      std::vector<int>& fromIndexes,
                                      int& toIndex)
{
    static bool loopDetected;loopDetected=false;
    static int numberInliers;
    static int j;j=0;

    std::vector<cv::DMatch> matches;
    std::vector<char> matchesMask;
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;

    relativePoses.resize(keyframes.size()-1);
    informationMatrices.resize(keyframes.size()-1);
    fromIndexes.resize(keyframes.size()-1);
    toIndex=keyframes.size()-1;

    for(int i=0;i<keyframes.size()-2;i++)
    {
        //If the last vertex pose is reasonably close to poses[i], check for loop closure
        if(!Miscellaneous::poseHasChanged(poses.back(),poses[i],0.4,30))
        {
            //Match the current keyframe against previous keyframes
            profiler.enter("Feature matching");
            matches.clear();
            matcher.match(descriptorsList[i],descriptorsList.back(),matches);
            profiler.leave("Feature matching");

            //Perform outlier rejection with the Fundamental or the Homography matrix
            matchesMask.clear();
            #if ENABLE_HOMOGRAPHY_OUTLIER_REMOVAL
            profiler.enter("Homography matrix outlier removal");
            numberInliers = matcher.outlierRemovalHomography(keypointsList[i],keypointsList.back(),matches,matchesMask,3.0);
            profiler.leave("Homography matrix outlier removal");
            #else
            profiler.enter("Fundamental matrix outlier removal");
            numberInliers = matcher.outlierRemovalFundamentalMat(keypointsList[i],keypointsList.back(),matches,matchesMask,3.0);
            profiler.leave("Fundamental matrix outlier removal");
            #endif

            //If the number of inliers is greater than a certain threshold, then try estimate the rigid transformation
            if(numberInliers>numberInliersThreshold)
            {
                points1.clear();
                points2.clear();
                matcher.get2DMatchedPoints(keypointsList[i],keypointsList.back(),matches,points1,points2,numberInliers,matchesMask);

                fromIndexes[j]=i;
                relativePoses[j]=Eigen::Matrix4f::Identity ();

                //Estimate the rigid transformation
                profiler.enter("Rigid transformation estimation");
                int numberInliersAux=transformationEstimator.estimateVisual3DRigidTransformation(points1,points2,keyframes[i]->pointCloudPtr,keyframes.back()->pointCloudPtr,relativePoses[j]);
                profiler.leave("Rigid transformation estimation");
                if(numberInliersAux==-1)
                {
                    //Not using outlier rejection
                }
                else
                {
                    //Using outlier rejection
                    numberInliers=numberInliersAux;
                }

                //If after the RANSAC rejection, there are still enough inliers
                //we consider that a loop has been detected
                if(numberInliers>numberInliersThreshold)
                {
                    loopDetected=true;

                    //Refine the rigid transformation with a ICP/GICP
                    #if ENABLE_RIGID_TRANSFORMATION_REFINEMENT
                    profiler.enter("Rigid transformation refinement ICP/GICP");
                    poseRefiner.refinePose(*keyframes[i],*keyframes.back(),relativePoses[j]);
                    profiler.leave("Rigid transformation refinement ICP/GICP");
                    #endif

                    //Set the loop relative pose
                    relativePoses[j]=relativePoses[j].inverse(); //SVD/ICP/GICP returns the inverse of the relative pose

                    //Set the loop information matrix
                    informationMatrices[j]=numberInliers*Eigen::Matrix<double,6,6>::Identity();

                    j++;
                }
            }
        }
    }

    relativePoses.resize(j);
    fromIndexes.resize(j);
    informationMatrices.resize(j);

    //Add the relative pose between previous keyframe and current keyframe
    relativePoses.push_back(relativePose);
    fromIndexes.push_back(toIndex-1);

    //Set the information matrix to the constraint with the previous keyframe
    profiler.enter("Feature matching (previous keyframe)");
    matches.clear();
    matcher.match(descriptorsList[descriptorsList.size()-2],descriptorsList.back(),matches);
    profiler.leave("Feature matching (previous keyframe)");
    matchesMask.clear();
    #if ENABLE_HOMOGRAPHY_OUTLIER_REMOVAL
    profiler.enter("Homography matrix outlier removal (previous keyframe)");
    numberInliers = matcher.outlierRemovalHomography(keypointsList[keypointsList.size()-2],keypointsList.back(),matches,matchesMask,3.0);
    profiler.leave("Homography matrix outlier removal (previous keyframe)");
    #else
    profiler.enter("Fundamental matrix outlier removal (previous keyframe)");
    numberInliers = matcher.outlierRemovalFundamentalMat(keypointsList[keypointsList.size()-2],keypointsList.back(),matches,matchesMask,3.0);
    profiler.leave("Fundamental matrix outlier removal (previous keyframe)");
    #endif
    if(numberInliers<=0){numberInliers=1;}
    informationMatrices.push_back(numberInliers*Eigen::Matrix<double,6,6>::Identity());

    #if ENABLE_DEBUG_KEYFRAME_ALIGNMENT
    static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
    grid.setLeafSize(0.025,0.025,0.025);

    //Debug: Save keyframe alignment
    for(int i=0;i<relativePoses.size();i++)
    {
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr alignedCloudPtr( new pcl::PointCloud<pcl::PointXYZRGBA> );
        Eigen::Matrix4f inverseRelativePose = relativePoses[i].inverse();
        pcl::transformPointCloud(*keyframes[fromIndexes[i]]->pointCloudPtr,*alignedCloudPtr,inverseRelativePose);
        alignedCloudPtr->header.frame_id = keyframes[toIndex]->pointCloudPtr->header.frame_id;

        for(int j=0;j<alignedCloudPtr->points.size();j++)
        {
            alignedCloudPtr->points[j].r=255;
            alignedCloudPtr->points[j].g=0;
            alignedCloudPtr->points[j].b=0;
        }

        *alignedCloudPtr += *keyframes[toIndex]->pointCloudPtr;
        pcl::PointCloud<pcl::PointXYZRGBA> downsampledAlignedCloud;
        grid.setInputCloud (alignedCloudPtr);
        grid.filter (downsampledAlignedCloud);
        pcl::io::savePCDFile(mrpt::format("../results/pcd_files/keyframes_%03i_%03i.pcd",fromIndexes[i],toIndex),downsampledAlignedCloud);
    }
    #endif

}

void KeyframeLoopDetector::getPoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& accPoses)
{
    accPoses = poses;
}

void KeyframeLoopDetector::getKeyframes(std::vector<FrameRGBD*>& keyframeVector)
{
   keyframeVector=keyframes;
}

void KeyframeLoopDetector::accumulateRelativePose(Eigen::Matrix4f& relPose)
{
    relativePose = relPose;
    poses.push_back(poses.back()*relativePose);
}

void KeyframeLoopDetector::getCurrentPose(Eigen::Matrix4f& pose)
{
    pose=poses.back();
}

void KeyframeLoopDetector::setPoses(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& newPoses)
{
    poses.clear();
    poses=newPoses;
}

void KeyframeLoopDetector::saveLoopProfiling(std::string fileName)
{
    profiler.saveToCSVFile(fileName);
}




