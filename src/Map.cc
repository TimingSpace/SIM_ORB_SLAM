/**
* This file is part of ORB-SLAM.
*
* Copyright (C) 2014 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <http://webdiis.unizar.es/~raulmur/orbslam/>
*
* ORB-SLAM is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM. If not, see <http://www.gnu.org/licenses/>.
*/

#include "../include/Map.h"

namespace ORB_SLAM
{

Map::Map()
{
    mbMapUpdated= false;
    mnMaxKFid = 0;
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
    mbMapUpdated=true;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    mspMapPoints.insert(pMP);
    mbMapUpdated=true;
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    mspMapPoints.erase(pMP);
    mbMapUpdated=true;
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    mspKeyFrames.erase(pKF);
    mbMapUpdated=true;
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    mvpReferenceMapPoints = vpMPs;
    mbMapUpdated=true;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

int Map::MapPointsInMap()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    return mspMapPoints.size();
}

int Map::KeyFramesInMap()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    return mvpReferenceMapPoints;
}

bool Map::isMapUpdated()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    return mbMapUpdated;
}

void Map::SetFlagAfterBA()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    mbMapUpdated=true;

}

void Map::ResetUpdated()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD
    mbMapUpdated=false;
}

unsigned int Map::GetMaxKFid()
{
    #ifndef ONE_THREAD
    boost::mutex::scoped_lock lock(mMutexMap);
    #endif // ONE_THREAD  
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
}

} //namespace ORB_SLAM
