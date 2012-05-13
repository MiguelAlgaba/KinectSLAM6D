#ifndef VISUAL_FEATURE_MATCHER
#define VISUAL_FEATURE_MATCHER

#include "opencv2/features2d/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

/*!Abstract class that specifies a method to perform 2D visual feature matching.*/
class VisualFeatureMatcher
{
protected:

public:
	/*!Performs descriptors matching and returns a vector of 2D matches. For every descriptor in descriptors1, the method computes the nearest descriptor in descriptors2.*/
	virtual void match(const cv::Mat& descriptors1,const cv::Mat& descriptors2,std::vector<cv::DMatch>& matches)=0;
};
#endif



