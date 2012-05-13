#include "../include/PointCloudViewer_MRPT.h"

PointCloudViewer_MRPT::PointCloudViewer_MRPT(const std::string &windowTitle)
{
	// Create window and prepare OpenGL object in the scene:
    win3D.Create(windowTitle,800,600);
    win3D.setWindowTitle(windowTitle);

	win3D.setCameraAzimuthDeg(140);
	win3D.setCameraElevationDeg(20);
	win3D.setCameraZoom(8.0);
	win3D.setFOV(90);
	win3D.setCameraPointingToPoint(2.5,0,0);

	gl_points = mrpt::opengl::CPointCloudColoured::Create();
	gl_points->setPointSize(2.5);

	{
		mrpt::opengl::COpenGLScenePtr &scene = win3D.get3DSceneAndLock();

		// Create the Opengl object for the point cloud:
		scene->insert( gl_points );
		scene->insert( mrpt::opengl::CGridPlaneXY::Create() );
		scene->insert( mrpt::opengl::stock_objects::CornerXYZ() );

		win3D.addTextMessage(5,5,
			mrpt::format("'o'/'i'-zoom out/in, ESC to exit"),
				mrpt::utils::TColorf(1,1,1), 110, mrpt::opengl::MRPT_GLUT_BITMAP_HELVETICA_18 );

		win3D.unlockAccess3DScene();
		win3D.repaint();
	}

	escPressed=false;
}

//PointCloudViewer_MRPT::~PointCloudViewer_MRPT(){}

void PointCloudViewer_MRPT::showPointCloud(pcl::PointCloud<pcl::PointXYZRGBA>& pointCloud)
{
    win3D.get3DSceneAndLock();
    gl_points->loadFromPointsMap(&pointCloud);
    win3D.unlockAccess3DScene();
    win3D.repaint();

	// Process possible keyboard commands:
	// --------------------------------------
	if (win3D.keyHit())
	{
		const int key = tolower( win3D.getPushedKey() );
		switch(key)
		{
			// Some of the keys are processed in this thread:
			case 'o':
				win3D.setCameraZoom( win3D.getCameraZoom() * 1.2 );
				win3D.repaint();
				break;
			case 'i':
				win3D.setCameraZoom( win3D.getCameraZoom() / 1.2 );
				win3D.repaint();
				break;
			case 27: // ESC
				escPressed=true;
				break;
			default:
				break;
		};
	}
}

bool PointCloudViewer_MRPT::isOpen()
{
    return win3D.isOpen() && !escPressed;
}
