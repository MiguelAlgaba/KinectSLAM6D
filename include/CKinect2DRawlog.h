#ifndef _CKINECT2DRAWLOG_
#define _CKINECT2DRAWLOG_

#include <mrpt/system.h>
#include <mrpt/slam/CObservation2DRangeScan.h>
#include <mrpt/slam/CObservation3DRangeScan.h>
#include <mrpt/slam/CRawlog.h>
#include <mrpt/utils/CFileGZInputStream.h>

/*!This class encapsulates the basic functionality of a 2D scan 
grabber from a rawlog file*/
using namespace mrpt::slam;
class CKinect2DRawlog
{
public:

	/*!Constructor*/
	CKinect2DRawlog(const std::string &rawlog_file);
	
	/*!Destructor*/
	~CKinect2DRawlog();

	/*!Initialize the connection with the Kinect sensor*/
	void init();

	/*!Start grabbing point clouds*/
	inline void startGrabbing(){}

	/*!Stop grabbing point clouds*/
	inline void stopGrabbing(){}

	/*!Get the 2D scan*/
	void get2DScan(mrpt::slam::CObservation2DRangeScan&);

	/*!Set maximum range*/
	inline void setMaximumRange(double range = 4){m_maxRange=range;}

private:

	/*!Observation 2D*/
	mrpt::slam::CObservation2DRangeScan m_scan2D;

	/*!Observation 3D*/
	mrpt::slam::CObservation3DRangeScanPtr m_obs3DPtr;

	/*!File stream to the rawlog dataset*/
	mrpt::utils::CFileGZInputStream*  m_dataset; 

	/*!Maximum range to consider a measure as valid*/
	double m_maxRange;

};
#endif

//#endif