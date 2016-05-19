#include <opencv2/opencv.hpp>
#include <string>
#include <iostream>
#include <fstream>
#include "../include/Frame.h"
#include "../include/ORBmatcher.h"
#include "../include/Initializer.h"
#include "../include/Map.h"
#include "../include/Optimizer.h"
#include "tracking.h"
using namespace cv;
using namespace ORB_SLAM;
using namespace std;
void LoadImages(const string &strPathToSequence, vector<string> &vstrImageFilenames, vector<double> &vTimestamps);

int main(int argn, char** argv)
{
	// initial path and parameter
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


    Map* mpMap = new Map();
    Tracking trackor=Tracking(mpMap);
    trackor.trackingImage(vstrImageFilenames,K,DistCoef);
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