#include "../include/CKinect2DRawlog.h"

CKinect2DRawlog::CKinect2DRawlog(const std::string &rawlog_file)
{
	//Open the rawlog file
    m_dataset = new mrpt::utils::CFileGZInputStream();
    if (!m_dataset->open(rawlog_file))
        throw std::runtime_error("Couldn't open rawlog dataset file for input...");

	// Set external images directory:
    mrpt::utils::CImage::IMAGES_PATH_BASE = mrpt::slam::CRawlog::detectImagesDirectory(rawlog_file);
}

CKinect2DRawlog::~CKinect2DRawlog()
{

}

void CKinect2DRawlog::init()
{

}

void CKinect2DRawlog::get2DScan(mrpt::slam::CObservation2DRangeScan& scan)
{
	bool grabbed=false;

	//Initialize the point cloud pointer with an empty point cloud
	m_scan2D.scan.resize(640);
	m_scan2D.validRange.resize(640);
	m_scan2D.aperture = 0.994837674; //The field-of-view of the scanner, in radians
	m_scan2D.rightToLeft = false; //Set the scan direction from left to right
	m_scan2D.sensorLabel = "Kinect2D"; //Set the sensor label

    while(!grabbed)
    {
        //Read observations until we get a CObservation3DRangeScan
        mrpt::slam::CObservationPtr obs;
        do
        {
            try
            {
                *m_dataset >> obs;
            }
            catch (std::exception &e)
            {
                throw std::runtime_error( std::string("\nError reading from dataset file (EOF?):\n")+std::string(e.what()) );
            }
		} while (!IS_CLASS(obs,CObservation3DRangeScan));

        // We have one observation:
		m_obs3DPtr = mrpt::slam::CObservation3DRangeScanPtr(obs);
        m_obs3DPtr->load(); // *Important* This is needed to load the range image if stored as a separate file.

		//Compute the 3D point cloud from depth image
		m_obs3DPtr->project3DPointsFromDepthImage();

		//   m_obs3DPtr->project3DPointsFromDepthImageInto(m_obs3DPtr, false /* without obs.sensorPose */);

        if (m_obs3DPtr->hasRangeImage &&
            m_obs3DPtr->hasIntensityImage)
        {

			//Get the central horizontal data from the point cloud
			for(int pIndex=0;pIndex<640;pIndex++)
			{
				//Identify valid and invalid data
				if(m_obs3DPtr->points3D_x[240*640+pIndex]!=0 &&
				   m_obs3DPtr->points3D_y[240*640+pIndex]!=0 &&
				   m_obs3DPtr->points3D_z[240*640+pIndex]!=0 &&
				   m_obs3DPtr->points3D_x[240*640+pIndex]<m_maxRange)
				{
					m_scan2D.scan[pIndex]=std::sqrt(std::pow(m_obs3DPtr->points3D_x[240*640+pIndex],2)+
													std::pow(m_obs3DPtr->points3D_y[240*640+pIndex],2));
					m_scan2D.validRange[pIndex]=1;

				}
				else
				{
					m_scan2D.validRange[pIndex]=0;
				}
			}

            //Retain the timestamp of the current frame
			m_scan2D.timestamp = m_obs3DPtr->timestamp;

            grabbed=true;
        }
    }

	//Return the current horizontal 2D scan
	scan = m_scan2D;
}
