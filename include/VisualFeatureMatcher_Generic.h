#ifndef VISUAL_FEATURE_MATCHER_GENERIC
#define VISUAL_FEATURE_MATCHER_GENERIC

#include "VisualFeatureMatcher.h"

/*!This class encapsulates the functionality of a generic 2D feature descriptor matcher using the OpenCV library.*/
class VisualFeatureMatcher_Generic : public VisualFeatureMatcher
{
protected:
    cv::Ptr<cv::DescriptorMatcher> descriptorMatcher;
    int matcherFilterType;
    enum { NONE_FILTER = 0, CROSS_CHECK_FILTER = 1 };
public:
	/*!Creates an instance of VisualFeatureMatcher_Generic given a descriptor matcher a std::string that specifies the matching algorithm.
		- "NoneFilter": Computes the nearest descriptors of descriptors1 in descriptors2 and returns the vector of matches.
		- "CrossCheckFilter": Computes the nearest descriptors of descriptors1 in descriptors2 (forward matching). Then it computes the nearest descriptors of descriptors2 in descriptors1 (backward matching). It then retains the matches that appears in both forward and backward matching.*/
	VisualFeatureMatcher_Generic(cv::Ptr<cv::DescriptorMatcher> matcher,
			             const std::string& matchingAlgorithm);
	
	/*!Performs descriptor matching using the Cross Check Filter algorithm. Computes the nearest descriptors of descriptors1 in descriptors2 (forward matching). Then it computes the nearest descriptors of descriptors2 in descriptors1 (backward matching). It then retains the matches that appears in both forward and backward matching.*/
	void crossCheckMatching(const cv::Mat& descriptors1,
				const cv::Mat& descriptors2,
				std::vector<cv::DMatch>& matches);

	/*!Performs descriptor matching. Computes the nearest descriptors of descriptors1 in descriptors2 and returns a vector of matches.*/
	void simpleMatching(const cv::Mat& descriptors1,
			    const cv::Mat& descriptors2,
			    std::vector<cv::DMatch>& matches);

	/*!Performs descriptor matching. Computes simpleMatching or crossCheckMatching depending on the name of the algorithm specified when creating the instance of VisualFeatureMatcher_Generic.*/
	void match(const cv::Mat& descriptors1,const cv::Mat& descriptors2,std::vector<cv::DMatch>& matches);

	/*!Performs outlier rejection with the homography matrix. This method computes the homography matrix that best fits the 2D matches using RANSAC and then rejects the 2D correspondences that doesn't fit the model to get robust 2D correspondences. This method returns the resulting number of inliers.*/        
	int outlierRemovalHomography(
	    const std::vector<cv::KeyPoint>& keypoints1,
            const std::vector<cv::KeyPoint>& keypoints2,
	    const std::vector<cv::DMatch>& matches,
            std::vector<char>& matchesMask,
            const double ransacInlierDistance=3.0); 

	/*!Performs outlier rejection with the fundamental matrix. This method computes the fundamental matrix that best fits the 2D matches using RANSAC and then rejects the 2D correspondences that doesn't fit the model to get robust 2D correspondences. This method returns the resulting number of inliers.*/ 
	int outlierRemovalFundamentalMat(
	    const std::vector<cv::KeyPoint>& keypoints1,
            const std::vector<cv::KeyPoint>& keypoints2,
	    const std::vector<cv::DMatch>& matches,
            std::vector<char>& matchesMask,
            const double ransacInlierDistance=3.0); 

	/*!This method takes the keypoints, matches and matchesMask to obtain two vectors of 2D points with the pairs of 2D correspondences of each match.*/
	void get2DMatchedPoints(
			const std::vector<cv::KeyPoint>& keypoints1,
			const std::vector<cv::KeyPoint>& keypoints2,
			const std::vector<cv::DMatch>& matches,
            		std::vector<cv::Point2f>& points1,
			std::vector<cv::Point2f>& points2,
			const int numberInliers,
            		const std::vector<char>& matchesMask);

    /*!This method takes the keypoints and matches to obtain two vectors of 2D points with the pairs of 2D correspondences of each match.*/
    void get2DMatchedPoints(
			const std::vector<cv::KeyPoint>& keypoints1,
			const std::vector<cv::KeyPoint>& keypoints2,
			const std::vector<cv::DMatch>& matches,
            		std::vector<cv::Point2f>& points1,
			std::vector<cv::Point2f>& points2);
};
#endif
