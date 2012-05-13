#include "../include/Visual3DRigidTransformationEstimator_RANSAC.h"

Visual3DRigidTransformationEstimator_RANSAC::Visual3DRigidTransformationEstimator_RANSAC(const double minValue,const double maxValue)
{
	minDepthValue=minValue;
	maxDepthValue=maxValue;
}

//Visual3DRigidTransformationEstimator_RANSAC::~Visual3DRigidTransformationEstimator_RANSAC(){}

void Visual3DRigidTransformationEstimator_RANSAC::setMinDepthValue(const double minValue)
{
	minDepthValue=minValue;
}

void Visual3DRigidTransformationEstimator_RANSAC::setMaxDepthValue(const double maxValue)
{
	maxDepthValue=maxValue;
}

//Get 3D correspondences with valid data
void Visual3DRigidTransformationEstimator_RANSAC::matchesWith3DValidData(
				const std::vector<cv::Point2f>& points2d1,
                const std::vector<cv::Point2f>& points2d2,
                const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr1,
				const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr2)
{

    static int numberValidPoints;
    static int point1X,point1Y,point2X,point2Y;
    static int pointIndex1,pointIndex2;
    static double depthPoint1,depthPoint2;

    for(int i=0;i<points2d1.size();i++)
    {
        point1X=points2d1[i].x;
        point1Y=points2d1[i].y;

        point2X=points2d2[i].x;
        point2Y=points2d2[i].y;

        pointIndex1=640*point1Y+point1X;
        pointIndex2=640*point2Y+point2X;

        if(pointIndex1<=0 ||  //Check if the idexes are invalid
           pointIndex1>=pointCloudPtr1->points.size() ||
           pointIndex2<=0 ||
           pointIndex2>=pointCloudPtr2->points.size())
        {
            (*correspondences)[i].index_query=-1;
            (*correspondences)[i].index_match=-1;
            (*correspondences)[i].distance=0;
        }
        else
        {
            depthPoint1=pointCloudPtr1->points[pointIndex1].z;
            depthPoint2=pointCloudPtr2->points[pointIndex2].z;

            if((minDepthValue<=depthPoint1) && (depthPoint1<=maxDepthValue) && //The first observation has valid depth data
               (minDepthValue<=depthPoint2) && (depthPoint2<=maxDepthValue))   //The second observation has valid depth data
            {
                //Check for valid (x,y,z) values
                if(pcl_isfinite (pointCloudPtr1->points[pointIndex1].x) &&
                   pcl_isfinite (pointCloudPtr1->points[pointIndex1].y) &&
                   pcl_isfinite (pointCloudPtr1->points[pointIndex1].z) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].x) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].y) &&
                   pcl_isfinite (pointCloudPtr2->points[pointIndex2].z))
                {
                    double distance = sqrt(pow(pointCloudPtr1->points[pointIndex1].x-
                                               pointCloudPtr2->points[pointIndex2].x,2)+
                                           pow(pointCloudPtr1->points[pointIndex1].y-
                                               pointCloudPtr2->points[pointIndex2].y,2)+
                                           pow(pointCloudPtr1->points[pointIndex1].z-
                                               pointCloudPtr2->points[pointIndex2].z,2));

                    (*correspondences)[i].index_query=pointIndex1;
                    (*correspondences)[i].index_match=pointIndex2;
                    (*correspondences)[i].distance=0;
                }
                else
                {
                    (*correspondences)[i].index_query=-1;
                    (*correspondences)[i].index_match=-1;
                    (*correspondences)[i].distance=0;

                }
            }
            else
            {
                (*correspondences)[i].index_query=-1;
                (*correspondences)[i].index_match=-1;
                (*correspondences)[i].distance=0;
            }
        }
    }

}


int Visual3DRigidTransformationEstimator_RANSAC::estimateVisual3DRigidTransformation(
		const std::vector<cv::Point2f>& points2d1,
		const std::vector<cv::Point2f>& points2d2,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr1,
		const pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr2,
		Eigen::Matrix4f& H)
{
    correspondences.reset(new pcl::Correspondences);
    correspondences->clear();
    correspondences->resize(points2d1.size());

    matchesWith3DValidData(points2d1,points2d2,pointCloudPtr1,pointCloudPtr2);

    corrsRejectorSAC.setInputCloud(pointCloudPtr1);
    corrsRejectorSAC.setTargetCloud(pointCloudPtr2);
    corrsRejectorSAC.setInlierThreshold(0.05);
    corrsRejectorSAC.setMaxIterations(500);
    corrsRejectorSAC.setInputCorrespondences(correspondences);
    boost::shared_ptr<pcl::Correspondences> correspondencesRejSAC (new pcl::Correspondences);
    corrsRejectorSAC.getCorrespondences(*correspondencesRejSAC);
    H = corrsRejectorSAC.getBestTransformation();

    //Return the number of RANSAC inliers
    return correspondencesRejSAC->size();
}






