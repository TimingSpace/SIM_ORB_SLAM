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
	// initial path and parameter
	Mat img1=imread("../image_2/000100.png",0);
	Mat img2=imread("../image_2/000103.png",0);
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

    // initial frame
	Frame mReferenceFrame=Frame(img1,0,mORBextractor,K,DistCoef);
	Frame mCurrentFrame=Frame(img2,1,mORBextractor,K,DistCoef);
	// test orb extraction
	// cout<<mReferenceFrame.mvKeys.size()<<endl;
	Mat img1Draw(img1.size(),img1.type()),img2Draw(img2.size(),img2.type());
	drawKeypoints(img1,mReferenceFrame.mvKeys,img1Draw,Scalar(255,0,0));
	drawKeypoints(img2,mCurrentFrame.mvKeys,img2Draw,Scalar(255,0,0));

	Initializer mInitializer=Initializer(mReferenceFrame);
	
	// orb matching
	ORBmatcher matcher(0.9,true);
	
	vector<int> mIniMatches;

	mIniMatches.resize(mReferenceFrame.mvKeys.size());

	vector<cv::Point2f> mPrevMatched;
	mPrevMatched.resize(mReferenceFrame.mvKeys.size());
	for(size_t i=0; i<mReferenceFrame.mvKeys.size(); i++)
            mPrevMatched[i]=mReferenceFrame.mvKeysUn[i].pt;
   	
   	// cout<<mPrevMatched[0].x<<" "<<mPrevMatched[0].y<<"  "<<mReferenceFrame.mvKeysUn[0].octave<<endl;
   	vector<size_t> vIndices2 = mCurrentFrame.GetFeaturesInArea(mPrevMatched[0].x,mPrevMatched[0].y, 100,0,7);
   	// for(int i=0;i<48;i++)
   	// 	cout<<mCurrentFrame.mGrid[i][i].size()<<endl; //error comes from here
    int nmatches = matcher.SearchForInitialization(mReferenceFrame,mCurrentFrame,mPrevMatched,mIniMatches,100);
    cout<<nmatches<<endl;
    Mat R21;
    Mat t21; 
    vector<cv::Point3f> vP3D;
    vector<bool> vbTriangulated;

    bool success=mInitializer.Initialize(mCurrentFrame,mIniMatches,R21,t21,vP3D,vbTriangulated);
    cout<<"initial success "<<success<<endl;
	imshow("img1",img1Draw);
	imshow("img2",img2Draw);
	waitKey();
	return 0;
}