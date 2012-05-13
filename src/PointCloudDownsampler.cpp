#include "../include/PointCloudDownsampler.h"

PointCloudDownsampler::PointCloudDownsampler(const int step)
{
    downsamplingStep = step;
}

//PointCloudDownsampler::~PointCloudDownsampler(){}

void PointCloudDownsampler::setDownsamplingStep(const int step)
{
    downsamplingStep=step;
}

void PointCloudDownsampler::downsamplePointCloud(
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& downsampledPointCloudPtr)
{
    static int j;j=0;
    std::vector<double> xV;
    std::vector<double> yV;
    std::vector<double> zV;

    downsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA> );
    downsampledPointCloudPtr->points.resize(640*480/downsamplingStep*downsamplingStep);
    for(int r=0;r<480;r=r+downsamplingStep)
    {
        for(int c=0;c<640;c=c+downsamplingStep)
        {
            int nPoints=0;
            xV.resize(downsamplingStep*downsamplingStep);
            yV.resize(downsamplingStep*downsamplingStep);
            zV.resize(downsamplingStep*downsamplingStep);

            for(int r2=r;r2<r+downsamplingStep;r2++)
            {
                for(int c2=c;c2<c+downsamplingStep;c2++)
                {
                    //Check if the point has valid data
                    if(pcl_isfinite (pointCloudPtr->points[r2*640+c2].x) &&
                       pcl_isfinite (pointCloudPtr->points[r2*640+c2].y) &&
                       pcl_isfinite (pointCloudPtr->points[r2*640+c2].z) &&
                       0.3<pointCloudPtr->points[r2*640+c2].z &&
                       pointCloudPtr->points[r2*640+c2].z<5)
                    {
                        //Create a vector with the x, y and z coordinates of the square region
                        xV[nPoints]=pointCloudPtr->points[r2*640+c2].x;
                        yV[nPoints]=pointCloudPtr->points[r2*640+c2].y;
                        zV[nPoints]=pointCloudPtr->points[r2*640+c2].z;

                        nPoints++;
                    }
                }
            }

            //Check if there are points in the region
            if(nPoints>0)
            {
                xV.resize(nPoints);
                yV.resize(nPoints);
                zV.resize(nPoints);

                //Compute the mean 3D point
                std::sort(xV.begin(),xV.end());
                std::sort(yV.begin(),yV.end());
                std::sort(zV.begin(),zV.end());

                pcl::PointXYZRGBA point;
                point.x=xV[nPoints/2];
                point.y=yV[nPoints/2];
                point.z=zV[nPoints/2];

                //Set the mean point as the representative point of the region
                downsampledPointCloudPtr->points[j]=point;
                j++;
            }
        }
    }
    downsampledPointCloudPtr->points.resize(j);
    downsampledPointCloudPtr->width=downsampledPointCloudPtr->size();
    downsampledPointCloudPtr->height=1;
}

void PointCloudDownsampler::downsamplePointCloudColor(
                pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& pointCloudPtr,
			    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr& downsampledPointCloudPtr)
{
    static int j;j=0;
    std::vector<double> xV;
    std::vector<double> yV;
    std::vector<double> zV;
    std::vector<double> rV;
    std::vector<double> gV;
    std::vector<double> bV;

    downsampledPointCloudPtr.reset(new pcl::PointCloud<pcl::PointXYZRGBA> );
    downsampledPointCloudPtr->points.resize(640*480/downsamplingStep*downsamplingStep);
    for(int r=0;r<480;r=r+downsamplingStep)
    {
        for(int c=0;c<640;c=c+downsamplingStep)
        {
            int nPoints=0;
            xV.resize(downsamplingStep*downsamplingStep);
            yV.resize(downsamplingStep*downsamplingStep);
            zV.resize(downsamplingStep*downsamplingStep);
            rV.resize(downsamplingStep*downsamplingStep);
            gV.resize(downsamplingStep*downsamplingStep);
            bV.resize(downsamplingStep*downsamplingStep);

            for(int r2=r;r2<r+downsamplingStep;r2++)
            {
                for(int c2=c;c2<c+downsamplingStep;c2++)
                {
                    //Check if the point has valid data
                    if(pcl_isfinite (pointCloudPtr->points[r2*640+c2].x) &&
                       pcl_isfinite (pointCloudPtr->points[r2*640+c2].y) &&
                       pcl_isfinite (pointCloudPtr->points[r2*640+c2].z) &&
                       0.3<pointCloudPtr->points[r2*640+c2].z &&
                       pointCloudPtr->points[r2*640+c2].z<5)
                    {
                        //Create a vector with the x, y and z coordinates of the square region and RGB info
                        xV[nPoints]=pointCloudPtr->points[r2*640+c2].x;
                        yV[nPoints]=pointCloudPtr->points[r2*640+c2].y;
                        zV[nPoints]=pointCloudPtr->points[r2*640+c2].z;
                        rV[nPoints]=pointCloudPtr->points[r2*640+c2].r;
                        gV[nPoints]=pointCloudPtr->points[r2*640+c2].g;
                        bV[nPoints]=pointCloudPtr->points[r2*640+c2].b;

                        nPoints++;
                    }
                }
            }

            if(nPoints>0)
            {
                xV.resize(nPoints);
                yV.resize(nPoints);
                zV.resize(nPoints);
                rV.resize(nPoints);
                gV.resize(nPoints);
                bV.resize(nPoints);

                //Compute the mean 3D point and mean RGB value
                std::sort(xV.begin(),xV.end());
                std::sort(yV.begin(),yV.end());
                std::sort(zV.begin(),zV.end());
                std::sort(rV.begin(),rV.end());
                std::sort(gV.begin(),gV.end());
                std::sort(bV.begin(),bV.end());

                pcl::PointXYZRGBA point;
                point.x=xV[nPoints/2];
                point.y=yV[nPoints/2];
                point.z=zV[nPoints/2];
                point.r=rV[nPoints/2];
                point.g=gV[nPoints/2];
                point.b=bV[nPoints/2];

                //Set the mean point as the representative point of the region
                downsampledPointCloudPtr->points[j]=point;
                j++;
            }
        }
    }
    downsampledPointCloudPtr->points.resize(j);
    downsampledPointCloudPtr->width=downsampledPointCloudPtr->size();
    downsampledPointCloudPtr->height=1;
}
