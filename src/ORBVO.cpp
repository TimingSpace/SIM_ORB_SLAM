#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include "../include/Frame.h"
#include "../include/ORBmatcher.h"
#include "../include/Initializer.h"
#include "../include/Map.h"
#include "../include/Optimizer.h"
using namespace cv;
using namespace ORB_SLAM;
using namespace std;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);
void CreateInitialMap(Frame &mInitialFrame, Frame &mCurrentFrame,vector<int> &mvIniMatches, std::vector<cv::Point3f> &mvIniP3D,Mat &Rcw,Mat &tcw,Map* mpMap);
bool TrackPreviousFrame(Frame& mCurrentFrame, Frame &mLastFrame, Map* mpMap);
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
    // if(mInitializer)
    //     delete mInitializer;
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
                    //delete mInitializer;// there must be a delete
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

		        
                CreateInitialMap(mInitialFrame,mCurrentFrame,mIniMatches,mIniP3D,Rcw,tcw,mpMap);
		        mLastFrame = Frame(mCurrentFrame);
                mTrackingState = WORKING;
             
                cout<<"Initialized successfully"<<endl;
		    }
            waitKey(10);
		    continue;
    	}
        else if(mTrackingState==WORKING)
        {
            cout<<mCurrentFrame.mvKeys.size()<<endl;
            

            bool trackSuccess=TrackPreviousFrame(mCurrentFrame,mLastFrame,mpMap);
            mLastFrame = Frame(mCurrentFrame);

            cout<<trackSuccess<<"    "<<mCurrentFrame.mTcw<<endl;
        }
        //cout<<mpMap->GetAllMapPoints().size()<<endl;
    	waitKey();
    }	
	return 0;
}
bool TrackPreviousFrame(Frame& mCurrentFrame, Frame &mLastFrame, Map* mpMap)
{
    ORBmatcher matcher(0.9,true);
    vector<MapPoint*> vpMapPointMatches;

    // Search first points at coarse scale levels to get a rough initial estimate
    int minOctave = 0;
    int maxOctave = mCurrentFrame.mvScaleFactors.size()-1;
    if(mpMap->KeyFramesInMap()>5)
        minOctave = maxOctave/2+1;
    imshow("LastFrame",mLastFrame.im);
    int nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,200,vpMapPointMatches,minOctave);
    cout<<nmatches<<endl;
    // If not enough matches, search again without scale constraint
    if(nmatches<10)
    {
        nmatches = matcher.WindowSearch(mLastFrame,mCurrentFrame,100,vpMapPointMatches,0);
        if(nmatches<10)
        {
            vpMapPointMatches=vector<MapPoint*>(mCurrentFrame.mvpMapPoints.size(),static_cast<MapPoint*>(NULL));
            nmatches=0;
        }
    }

    mLastFrame.mTcw.copyTo(mCurrentFrame.mTcw);
    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    // If enough correspondeces, optimize pose and project points from previous frame to search more correspondences
    if(nmatches>=10)
    {
        // Optimize pose with correspondences
        Optimizer::PoseOptimization(&mCurrentFrame);

        for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
            if(mCurrentFrame.mvbOutlier[i])
            {
                mCurrentFrame.mvpMapPoints[i]=NULL;
                mCurrentFrame.mvbOutlier[i]=false;
                nmatches--;
            }

        // Search by projection with the estimated pose
        nmatches += matcher.SearchByProjection(mLastFrame,mCurrentFrame,15,vpMapPointMatches);
    }
    else //Last opportunity
        nmatches = matcher.SearchByProjection(mLastFrame,mCurrentFrame,50,vpMapPointMatches);


    mCurrentFrame.mvpMapPoints=vpMapPointMatches;

    if(nmatches<10)
        return false;

    // Optimize pose again with all correspondences
    Optimizer::PoseOptimization(&mCurrentFrame);

    // Discard outliers
    for(size_t i =0; i<mCurrentFrame.mvbOutlier.size(); i++)
        if(mCurrentFrame.mvbOutlier[i])
        {
            mCurrentFrame.mvpMapPoints[i]=NULL;
            mCurrentFrame.mvbOutlier[i]=false;
            nmatches--;
        }

    return nmatches>=10;
}
void CreateInitialMap(Frame &mInitialFrame, Frame &mCurrentFrame,vector<int> &mvIniMatches, std::vector<cv::Point3f> &mvIniP3D,Mat &Rcw,Mat &tcw,Map* mpMap)
{
    // Set Frame Poses
    mInitialFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    mCurrentFrame.mTcw = cv::Mat::eye(4,4,CV_32F);
    Rcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).colRange(0,3));
    tcw.copyTo(mCurrentFrame.mTcw.rowRange(0,3).col(3));
    //cout<<"0000000000000 "<<endl;
    // Create KeyFrames
    KeyFrame* pKFini = new KeyFrame(mInitialFrame,mpMap);
    KeyFrame* pKFcur = new KeyFrame(mCurrentFrame,mpMap);
    //cout<<"1111111111111 "<<endl;

    // Insert KFs in the map
    mpMap->AddKeyFrame(pKFini);
    mpMap->AddKeyFrame(pKFcur);
    //cout<<"222222222222"<<endl;

    // Create MapPoints and asscoiate to keyframes
    for(size_t i=0; i<mvIniMatches.size();i++)
    {
        if(mvIniMatches[i]<0)
            continue;

        //Create MapPoint.
        cv::Mat worldPos(mvIniP3D[i]);
        //cout<<"33330"<<endl;
        MapPoint* pMP = new MapPoint(worldPos,pKFcur,mpMap);
        //cout<<"33330.5"<<endl;
        pKFini->AddMapPoint(pMP,i);
        //cout<<"33330.8"<<endl;
        pKFcur->AddMapPoint(pMP,mvIniMatches[i]);
        //cout<<"33331"<<endl;
        pMP->AddObservation(pKFini,i);
        pMP->AddObservation(pKFcur,mvIniMatches[i]);
        //cout<<"33332"<<endl;
        pMP->ComputeDistinctiveDescriptors();
        pMP->UpdateNormalAndDepth();
        //cout<<"33333"<<endl;
        //Fill Current Frame structure
        mCurrentFrame.mvpMapPoints[mvIniMatches[i]] = pMP;

        //Add to Map
        mpMap->AddMapPoint(pMP);

    }
    //cout<<"44444444444"<<endl;

    // // Update Connections
    pKFini->UpdateConnections();
    pKFcur->UpdateConnections();

    // // Bundle Adjustment

    // Optimizer::GlobalBundleAdjustemnt(mpMap,20);

    // // Set median depth to 1
    float medianDepth = pKFini->ComputeSceneMedianDepth(2);
    float invMedianDepth = 1.0f/medianDepth;

    // if(medianDepth<0 || pKFcur->TrackedMapPoints()<100)
    // {
    //     //ROS_INFO("Wrong initialization, reseting...");
    //     Reset();
    //     return;
    // }

    // // Scale initial baseline
    cv::Mat Tc2w = pKFcur->GetPose();
    Tc2w.col(3).rowRange(0,3) = Tc2w.col(3).rowRange(0,3)*invMedianDepth;
    pKFcur->SetPose(Tc2w);

    // // Scale points
    vector<MapPoint*> vpAllMapPoints = pKFini->GetMapPointMatches();
    for(size_t iMP=0; iMP<vpAllMapPoints.size(); iMP++)
    {
        if(vpAllMapPoints[iMP])
        {
            MapPoint* pMP = vpAllMapPoints[iMP];
            pMP->SetWorldPos(pMP->GetWorldPos()*invMedianDepth);
        }
    }

    // mpLocalMapper->InsertKeyFrame(pKFini);
    // mpLocalMapper->InsertKeyFrame(pKFcur);

    mCurrentFrame.mTcw = pKFcur->GetPose().clone();
    // mLastFrame = Frame(mCurrentFrame);
    // mnLastKeyFrameId=mCurrentFrame.mnId;
    // mpLastKeyFrame = pKFcur;

    // mvpLocalKeyFrames.push_back(pKFcur);
    // mvpLocalKeyFrames.push_back(pKFini);
    // mvpLocalMapPoints=mpMap->GetAllMapPoints();
    // mpReferenceKF = pKFcur;

    // mpMap->SetReferenceMapPoints(mvpLocalMapPoints);

    // mpMapPublisher->SetCurrentCameraPose(pKFcur->GetPose());

    // mState=WORKING;
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