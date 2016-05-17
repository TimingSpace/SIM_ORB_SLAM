#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include "../include/Frame.h"
#include "../include/ORBmatcher.h"
#include "../include/Initializer.h"
using namespace cv;
using namespace ORB_SLAM;
using namespace std;
int main()
{

	Mat img1=imread("../image_2/000000.png",0);
	Mat img2=imread("../image_2/000001.png",0);
	ORBextractor *mORBextractor = new ORBextractor();
	Mat K = cv::Mat::eye(3,3,CV_32F);
    K.at<float>(0,0) = 718.1856;
    K.at<float>(1,1) = 718.1856;
    K.at<float>(0,2) = 607.1928;
    K.at<float>(1,2) = 185.2157;

    cv::Mat DistCoef(4,1,CV_32F);
    DistCoef.at<float>(0) = 0;
    DistCoef.at<float>(1) = 0;
    DistCoef.at<float>(2) = 0;
    DistCoef.at<float>(3) = 0;

	Frame mReferenceFrame=Frame(img1,0,mORBextractor,K,DistCoef);
	Frame mCurrentFrame=Frame(img2,1,mORBextractor,K,DistCoef);
	cout<<mReferenceFrame.mvKeys.size()<<endl;

	Initializer mInitializer=Initializer(mReferenceFrame);
	

	ORBmatcher matcher(0.9,true);
	vector<cv::Point2f> mPrevMatched;
	vector<int> mIniMatches;
	cout<<"nmatches"<<endl;
    int nmatches = matcher.SearchForInitialization(mReferenceFrame,mCurrentFrame,mPrevMatched,mIniMatches,100);
    cout<<nmatches<<endl;
    Mat R21;
    Mat t21; 
    vector<cv::Point3f> vP3D;
    vector<bool> vbTriangulated;
    mInitializer.Initialize(mCurrentFrame,mIniMatches,R21,t21,vP3D,vbTriangulated);
	imshow("img1",img1);
	waitKey();
	return 0;
}