#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

/*!
 Abstract class PointCloudViewer
 */
class PointCloudViewer
{

public:
    virtual void showPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>&)=0;
    virtual bool isOpen()=0;
};

