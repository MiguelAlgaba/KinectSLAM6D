#include "../include/Miscellaneous.h"

namespace Miscellaneous
{
    double rad2deg(const double angle)
    {
        static double ratio = 180.0 / 3.141592653589793238;
        return angle * ratio;
    }

	bool poseHasChanged(Eigen::Matrix4f& H1,Eigen::Matrix4f& H2,
		            double traslationThres,double rotationThres)
	{
	    static bool hasChanged;
	    static double yaw1,pitch1,roll1,yaw2,pitch2,roll2;
	    static double yawAngle,yawDistance,pitchAngle,pitchDistance,rollAngle,rollDistance;
	    static double traslation;

	    traslation=sqrt(pow(H1(0,3)-H2(0,3),2)+pow(H1(1,3)-H2(1,3),2)+pow(H1(2,3)-H2(2,3),2));

	    yaw1 = atan2f(H1(1,0),H1(0,0));
	    pitch1 = asinf(-H1(2,0));
	    roll1 = atan2f(H1(2,1),H1(2,2));

	    yaw2 = atan2f(H2(1,0),H2(0,0));
	    pitch2 = asinf(-H2(2,0));
	    roll2 = atan2f(H2(2,1),H2(2,2));

	    yawAngle = std::max(yaw1,yaw2)-std::min(yaw1,yaw2);
	    yawDistance = Miscellaneous::rad2deg(std::min(yawAngle,2*M_PI-yawAngle));

	    pitchAngle = std::max(pitch1,pitch2)-std::min(pitch1,pitch2);
	    pitchDistance = Miscellaneous::rad2deg(std::min(pitchAngle,2*M_PI-pitchAngle));

	    rollAngle = std::max(roll1,roll2)-std::min(roll1,roll2);
	    rollDistance = Miscellaneous::rad2deg(std::min(rollAngle,2*M_PI-rollAngle));

	    if(traslation>traslationThres || yawDistance+pitchDistance+rollDistance>rotationThres)
	    {
		hasChanged=true;
	    }
	    else
	    {
		hasChanged=false;
	    }
	    return hasChanged;
	}

	void printRelativePosesAndRelativeLoopPoses
      (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& accPoses,
       std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& relativePoses,
       std::vector<int>& fromIndexes,
       int toIndex)
	{
        for(int i=0;i<relativePoses.size();i++)
        {
            std::cout<<"Hacc_"<<fromIndexes[i]<<","<<toIndex<<std::endl;
            std::cout<<accPoses[fromIndexes[i]].inverse()*accPoses.back()<<std::endl;
            std::cout<<"H_"<<fromIndexes[i]<<","<<toIndex<<std::endl;
            std::cout<<relativePoses[i]<<std::endl;
            std::cout<<"---------------------------------------------"<<std::endl;
        }
	}

    void printAccumulatedPosesAndAccumulatedLoopPoses
      (std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& accPoses,
       std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& accLoopPoses)
    {
        for(int i=0;i<accPoses.size();i++)
        {
            std::cout<<"P_"<<i<<std::endl;
            std::cout<<accPoses[i]<<std::endl;
            std::cout<<"Ploop_"<<i<<std::endl;
            std::cout<<accLoopPoses[i]<<std::endl;
            std::cout<<"---------------------------------------------"<<std::endl;
        }
    }

    void generateGlobalMapPtr(std::vector<FrameRGBD*>& keyframes,
                              std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& accumulatedPoses,
                              pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& globalMapPtr)
    {
        static pcl::VoxelGrid<pcl::PointXYZRGBA> grid;
        grid.setLeafSize(0.01,0.01,0.01);

        PointCloudDownsampler downsampler = PointCloudDownsampler(4);

        pcl::PointCloud<pcl::PointXYZRGBA> globalMap;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keyframe;
        pcl::PointCloud<pcl::PointXYZRGBA>::Ptr keyframeDownsampled;

        for(int keyframeIdx=0;keyframeIdx<keyframes.size();keyframeIdx++)
        {
            pcl::PointCloud<pcl::PointXYZRGBA> alignedCloud;

            keyframe = keyframes[keyframeIdx]->pointCloudPtr;
            downsampler.downsamplePointCloudColor(keyframe,keyframeDownsampled);
            pcl::transformPointCloud(*keyframeDownsampled,alignedCloud,accumulatedPoses[keyframeIdx]);

            globalMapPtr->header.frame_id = alignedCloud.header.frame_id;
            *globalMapPtr += alignedCloud;
            if(keyframeIdx%5==0 || keyframeIdx==keyframes.size()-1)
            {
                globalMap.clear();
                grid.setInputCloud (globalMapPtr);
                grid.filter (globalMap);
                globalMapPtr->clear();
                *globalMapPtr = globalMap;
            }
        }
    }

    void saveMatrix(Eigen::Matrix4f& M,const char* fileName)
    {
        std::ofstream matrixFile;
        matrixFile.open (fileName);
        matrixFile << "[" << M(0,0) << ", " << M(0,1) << ", " << M(0,2) << ", " << M(0,3) << ";\n";
        matrixFile << M(1,0) << ", " << M(1,1) << ", " << M(1,2) << ", " << M(1,3) << ";\n";
        matrixFile << M(2,0) << ", " << M(2,1) << ", " << M(2,2) << ", " << M(2,3) << ";\n";
        matrixFile << M(3,0) << ", " << M(3,1) << ", " << M(3,2) << ", " << M(3,3) << "]";
        matrixFile.close();
    }

    void saveTrajectory(std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f > >& poses,
                        std::vector<FrameRGBD*>& keyframes,
                        const char* fileName)
    {
        double yaw,pitch,roll;
        std::ofstream trajectoryFile;

        trajectoryFile.open(fileName);

        trajectoryFile << "# estimated trajectory\n# timestamp tx ty tz qx qy qz qw\n";

        for(int keyframeIdx=0;keyframeIdx<keyframes.size();keyframeIdx++)
        {
            //Save pose timestamp
            trajectoryFile<<keyframes[keyframeIdx]->timeStamp/1000<<"."<<keyframes[keyframeIdx]->timeStamp-(keyframes[keyframeIdx]->timeStamp/1000)*1000<<" ";

            //Save the traslation and quaternion
            yaw = atan2f(poses[keyframeIdx](1,0),poses[keyframeIdx](0,0));
            pitch = asinf(-poses[keyframeIdx](2,0));
            roll = atan2f(poses[keyframeIdx](2,1),poses[keyframeIdx](2,2));

            trajectoryFile << poses[keyframeIdx](0,3) << " " <<
                              poses[keyframeIdx](1,3) << " " <<
                              poses[keyframeIdx](2,3) << " " <<
                              sin(roll/2)*cos(pitch/2)*cos(yaw/2)-cos(roll/2)*sin(pitch/2)*sin(yaw/2) << " " <<
                              cos(roll/2)*sin(pitch/2)*cos(yaw/2)+sin(roll/2)*cos(pitch/2)*sin(yaw/2) << " " <<
                              cos(roll/2)*cos(pitch/2)*sin(yaw/2)-sin(roll/2)*sin(pitch/2)*cos(yaw/2) << " " <<
                              cos(roll/2)*cos(pitch/2)*cos(yaw/2)+sin(roll/2)*sin(pitch/2)*sin(yaw/2) << std::endl;


        }

        trajectoryFile.close();
    }

}
