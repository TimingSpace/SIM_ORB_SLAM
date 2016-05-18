#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include "../include/Frame.h"
#include "../include/ORBmatcher.h"
#include "../include/Initializer.h"
#include "../include/Map.h"
using namespace cv;
using namespace ORB_SLAM;
using namespace std;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
int main(int argn, char** argv)
{
	// initial path and parameter
	Mat img1=imread("../00/image_0/000000.png",0);
	Mat img2=imread("../00/image_0/000002.png",0);
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

    //loading frame
    vector<string> vstrImageFilenames;
    vector<double> vTimestamps;
    cout << endl << "start loading images" << endl;
    LoadImages(string(argv[1]), vstrImageFilenames, vTimestamps);
    cout << endl << "images loaded" << endl;

    //tracking state
    enum eTrackingState{
        SYSTEM_NOT_READY=-1,
        NO_IMAGES_YET=0,
        NOT_INITIALIZED=1,
        INITIALIZING=2,
        WORKING=3,
        LOST=4
    };
    eTrackingState mTrackingState=NOT_INITIALIZED;

    //tracking
    Frame mReferenceFrame,mInitialFrame,mLastFrame;
    Frame mCurrentFrame;
    Initializer *mInitializer;
    vector<int> mIniMatches;
    // test 
    if(mInitializer)
        delete mInitializer;
    //end test
	mIniMatches.resize(mReferenceFrame.mvKeys.size());

	vector<cv::Point2f> mPrevMatched;
	std::vector<cv::Point3f> mIniP3D; // 3D point obtained by initilization
    
    Map* mpMap = new Map();
    for(int i=0;i<vstrImageFilenames.size();i++)
    {
    	Mat img=imread(vstrImageFilenames[i],0);
    	imshow("img",img);
    	mCurrentFrame=Frame(img,0,mORBextractor,K,DistCoef);
    	if(mTrackingState==NOT_INITIALIZED)
    	{
    		if(mCurrentFrame.mvKeys.size()>100)
		    {
		        mInitialFrame = Frame(mCurrentFrame);
		        mLastFrame = Frame(mCurrentFrame);
		        mPrevMatched.resize(mCurrentFrame.mvKeysUn.size());

		        for(size_t i=0; i<mCurrentFrame.mvKeysUn.size(); i++)
		            mPrevMatched[i]=mCurrentFrame.mvKeysUn[i].pt;
		        if(mInitializer)
                {
                    //delete mInitializer;
                }
		        mInitializer =  new Initializer(mCurrentFrame,1.0,200);
		        mTrackingState = INITIALIZING;
		    }
            waitKey(10);
		    continue;
    	}
    	else if(mTrackingState==INITIALIZING)
    	{
    		if(mCurrentFrame.mvKeys.size()<=100)
		    {
		        fill(mIniMatches.begin(),mIniMatches.end(),-1);
		        mTrackingState = NOT_INITIALIZED;
                
		        continue;
		    }    

		    // Find correspondences
		    ORBmatcher matcher(0.9,true);
		    int nmatches = matcher.SearchForInitialization(mInitialFrame,mCurrentFrame,mPrevMatched,mIniMatches,100);

		    // Check if there are enough correspondences
		    if(nmatches<100)
		    {
		        mTrackingState = NOT_INITIALIZED;
		        continue;
		    }  

		    cv::Mat Rcw; // Current Camera Rotation
		    cv::Mat tcw; // Current Camera Translation
		    vector<bool> vbTriangulated; // Triangulated Correspondences (mvIniMatches)

		    if(mInitializer->Initialize(mCurrentFrame, mIniMatches, Rcw, tcw, mIniP3D, vbTriangulated))
		    {
		        for(size_t i=0, iend=mIniMatches.size(); i<iend;i++)
		        {
		            if(mIniMatches[i]>=0 && !vbTriangulated[i])
		            {
		                mIniMatches[i]=-1;
		                nmatches--;
		            }           
		        }

		        cout<<"Initialized successfully"<<endl;
		        mTrackingState = WORKING;

		        //CreateInitialMap(Rcw,tcw);
		    }
            waitKey(10);
		    continue;
    	}
    	waitKey();
    }	
	return 0;
}

// Load Image Path and names It is a function from ORB SLAM
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps)
{
    ifstream fTimes;
    string strPathTimeFile = strPathToSequence + "/times.txt";
    fTimes.open(strPathTimeFile.c_str());
    while(!fTimes.eof())
    {
        string s;
        getline(fTimes,s);
        if(!s.empty())
        {
            stringstream ss;
            ss << s;
            double t;
            ss >> t;
            vTimestamps.push_back(t);
        }
    }

    string strPrefixLeft = strPathToSequence + "/image_0/";

    const int nTimes = vTimestamps.size();
    vstrImageFilenames.resize(nTimes);

    for(int i=0; i<nTimes; i++)
    {
        stringstream ss;
        ss << setfill('0') << setw(6) << i;
        vstrImageFilenames[i] = strPrefixLeft + ss.str() + ".png";
    }
}