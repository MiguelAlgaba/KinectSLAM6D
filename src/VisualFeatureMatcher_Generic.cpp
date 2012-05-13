#include "../include/VisualFeatureMatcher_Generic.h"
#include <string>

VisualFeatureMatcher_Generic::VisualFeatureMatcher_Generic(
			cv::Ptr<cv::DescriptorMatcher> descMatcher,
			const std::string& matcherFilterName)
{
    descriptorMatcher=descMatcher;
    if( matcherFilterName == "NoneFilter" )
    {
        matcherFilterType=NONE_FILTER;
    }
    else if ( matcherFilterName == "CrossCheckFilter" )
    {
        matcherFilterType=CROSS_CHECK_FILTER;
    }
    else
    {
        CV_Error(CV_StsBadArg, "Invalid filter name");
    }
}

//VisualFeatureMatcher_Generic::~VisualFeatureMatcher_Generic(){}

void VisualFeatureMatcher_Generic::crossCheckMatching(
				const cv::Mat& descriptors1,
				const cv::Mat& descriptors2,
				std::vector<cv::DMatch>& matches)
{
    static int knn=1;
    matches.clear();
    std::vector<std::vector<cv::DMatch> > matches12, matches21;
    descriptorMatcher->knnMatch( descriptors1, descriptors2, matches12, knn );
    descriptorMatcher->knnMatch( descriptors2, descriptors1, matches21, knn );

    for( size_t m = 0; m < matches12.size(); m++ )
    {
        bool findCrossCheck = false;
        for( size_t fk = 0; fk < matches12[m].size(); fk++ )
        {
            cv::DMatch forward = matches12[m][fk];

            for( size_t bk = 0; bk < matches21[forward.trainIdx].size(); bk++ )
            {
                cv::DMatch backward = matches21[forward.trainIdx][bk];
                if( backward.trainIdx == forward.queryIdx )
                {
                    matches.push_back(forward);
                    findCrossCheck = true;
                    break;
                }
            }
            if( findCrossCheck ) break;
        }
    }
}

void VisualFeatureMatcher_Generic::simpleMatching(
				const cv::Mat& descriptors1,
				const cv::Mat& descriptors2,
				std::vector<cv::DMatch>& matches)
{
    descriptorMatcher->match( descriptors1, descriptors2, matches );
}

void VisualFeatureMatcher_Generic::match(
             const cv::Mat& descriptors1,
             const cv::Mat& descriptors2,
             std::vector<cv::DMatch>& matches)
{
    if(matcherFilterType==CROSS_CHECK_FILTER)
    {
        crossCheckMatching(descriptors1,descriptors2,matches);
    }
    else
    {
        simpleMatching(descriptors1,descriptors2,matches);
    }
}

int VisualFeatureMatcher_Generic::outlierRemovalHomography(
	  const std::vector<cv::KeyPoint>& keypoints1,
	  const std::vector<cv::KeyPoint>& keypoints2,
	  const std::vector<cv::DMatch>& matches,
	  std::vector<char>& matchesMask,
      const double ransacReprojThreshold)
{
    static cv::Mat H12;
    static int numberInliers;numberInliers=0;

    if(matches.size()>=4) //If there are enough matches to perform findHomography, do it
    {
        std::vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
        for( size_t i = 0; i < matches.size(); i++ )
        {
            queryIdxs[i] = matches[i].queryIdx;
            trainIdxs[i] = matches[i].trainIdx;
        }

        std::vector<cv::Point2f> points1;
        std::vector<cv::Point2f> points2;
        cv::KeyPoint::convert(keypoints1, points1, queryIdxs);
        cv::KeyPoint::convert(keypoints2, points2, trainIdxs);

        H12 = cv::findHomography( cv::Mat(points1), cv::Mat(points2), CV_RANSAC, ransacReprojThreshold );

        matchesMask.resize( matches.size(), 0 );
        cv::Mat points1Transformed; cv::perspectiveTransform(cv::Mat(points1), points1Transformed, H12);

        //For each correspondence x_i <-> x_i', if distance(x_i',H*x_i)<threshold, then consider the correspondence as an inlier
        double maxInlierDist = ransacReprojThreshold < 0 ? 3 : ransacReprojThreshold;
        for( size_t i1 = 0; i1 < points2.size(); i1++ )
        {
            if( norm(points2[i1] - points1Transformed.at<cv::Point2f>((int)i1,0)) <= maxInlierDist ) // inlier
            {
                matchesMask[i1] = 1;
                numberInliers++;
            }
        }
    }
    else //If there are very few matches, return -1
    {
        numberInliers=-1;
    }

    return numberInliers;
}

int VisualFeatureMatcher_Generic::outlierRemovalFundamentalMat(
	  const std::vector<cv::KeyPoint>& keypoints1,
	  const std::vector<cv::KeyPoint>& keypoints2,
	  const std::vector<cv::DMatch>& matches,
	  std::vector<char>& matchesMask,
      const double ransacReprojThreshold)
{
    static cv::Mat F12;
    static int numberInliers;numberInliers=0;

    if(matches.size()>=8) //If there are enough matches to perform findFundamentalMat, do it
    {
        std::vector<int> queryIdxs( matches.size() ), trainIdxs( matches.size() );
        for( size_t i = 0; i < matches.size(); i++ )
        {
            queryIdxs[i] = matches[i].queryIdx;
            trainIdxs[i] = matches[i].trainIdx;
        }

        std::vector<cv::Point2f> points1;
        std::vector<cv::Point2f> points2;
        cv::KeyPoint::convert(keypoints1, points1, queryIdxs);
        cv::KeyPoint::convert(keypoints2, points2, trainIdxs);

        cv::Mat matchesMaskMat;
        F12=cv::findFundamentalMat(cv::Mat(points1),cv::Mat(points2),CV_FM_RANSAC,ransacReprojThreshold, 0.99,matchesMaskMat);

        matchesMask.resize( matches.size(), 0 );

        //For each correspondence x_i <-> x_i', if x_i'*F*x_i~=0, then consider the correspondence as an inlier
        cv::Mat point1 = cv::Mat(3,1,CV_64F);
        cv::Mat point2 = cv::Mat(3,1,CV_64F);
        cv::Mat d;
        cv::Mat l;
        for( size_t i1 = 0; i1 < points2.size(); i1++ )
        {
            if(matchesMaskMat.at<char>(i1)==1)
            {
                point1.at<double>(0) = points1[i1].x;
                point1.at<double>(1) = points1[i1].y;
                point1.at<double>(2) = 1.0;
                point2.at<double>(0) = points2[i1].x;
                point2.at<double>(1) = points2[i1].y;
                point2.at<double>(2) = 1.0;

                l=F12*point1;
                l=l/(std::sqrt(std::pow(l.at<double>(0),2)+std::pow(l.at<double>(0),2)));
                d = point2.t()*l;
                if(std::abs(d.at<double>(0))<=2)//Inlier
                {
                    matchesMask[i1]=1;
                    numberInliers++;
                }
            }
        }
    }
    else //If there are very few matches, return -1
    {
        numberInliers=-1;
    }

    return numberInliers;
}

void VisualFeatureMatcher_Generic::get2DMatchedPoints(
	     const std::vector<cv::KeyPoint>& keypoints1,
	     const std::vector<cv::KeyPoint>& keypoints2,
	     const std::vector<cv::DMatch>& matches,
         std::vector<cv::Point2f>& points1,
	     std::vector<cv::Point2f>& points2,
	     const int numberInliers,
	     const std::vector<char>& matchesMask)
{
    static int index1,index2;
    points1.clear();
    points2.clear();

    //get good matched points
	points1.resize(numberInliers);
	points2.resize(numberInliers);
	int j=0;
    for(int i=0;i<matches.size();i++)
    {
        index1=matches[i].queryIdx;
        index2=matches[i].trainIdx;

        if(matchesMask[i]==1 &&//is a good match
           keypoints1[index1].pt.x>=0 &&
           keypoints1[index1].pt.x<640 &&
           keypoints1[index1].pt.y>=0 &&
           keypoints1[index1].pt.y<480 &&
           keypoints2[index2].pt.x>=0 &&
           keypoints2[index2].pt.x<640 &&
           keypoints2[index2].pt.y>=0 &&
           keypoints2[index2].pt.y<480) //Valid 2d point
        {
            points1[j] = keypoints1[index1].pt;
            points2[j] = keypoints2[index2].pt;
            j++;
        }
    }
    points1.resize(j);
	points2.resize(j);
}

void VisualFeatureMatcher_Generic::get2DMatchedPoints(
	     const std::vector<cv::KeyPoint>& keypoints1,
	     const std::vector<cv::KeyPoint>& keypoints2,
	     const std::vector<cv::DMatch>& matches,
         std::vector<cv::Point2f>& points1,
	     std::vector<cv::Point2f>& points2)
{
    points1.clear();
    points2.clear();

    //get all points
	points1.resize(keypoints1.size());
	points2.resize(keypoints2.size());
    for(int i=0;i<matches.size();i++)
    {
        points1[i] = keypoints1[matches[i].queryIdx].pt;
        points2[i] = keypoints2[matches[i].trainIdx].pt;
    }
}






