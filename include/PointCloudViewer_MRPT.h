#include <mrpt/maps.h>
#include <mrpt/gui.h>
#include <mrpt/slam/PCL_adapters.h>
#include <string>

#include "PointCloudViewer.h"

class PointCloudViewer_MRPT : public PointCloudViewer
{
private:
    mrpt::gui::CDisplayWindow3D win3D;
    mrpt::opengl::CPointCloudColouredPtr gl_points;
    mrpt::opengl::COpenGLViewportPtr viewInt;
    bool escPressed;

public:
  PointCloudViewer_MRPT(const std::string &);
  void showPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>&);
  bool isOpen();
};
