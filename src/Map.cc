/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "Map.h"

#include<mutex>

namespace ORB_SLAM2
{

Map::Map():mnMaxKFid(0),mnBigChangeIdx(0)
{
}

void Map::AddKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.insert(pKF);
    if(pKF->mnId>mnMaxKFid)
        mnMaxKFid=pKF->mnId;
}

void Map::AddMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.insert(pMP);
}

// Add by liujiamin
void Map::AddMapAruco(MapAruco* pMA)
{
    unique_lock<mutex> lock(mMutexMap);
    // cout<<"Now is add MapAruco"<<endl;
    mspMapArucos.insert(pMA);
}

void Map::AddArucoMapPoint(MapPoint* pAMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspArucoMapPoints.insert(pAMP);
}

void Map::AddArucoMapPointsID(int id)
{
    unique_lock<mutex> lock(mMutexMap);
    msAMPsid.insert(id);
}

void Map::EraseMapPoint(MapPoint *pMP)
{
    unique_lock<mutex> lock(mMutexMap);
    mspMapPoints.erase(pMP);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::EraseKeyFrame(KeyFrame *pKF)
{
    unique_lock<mutex> lock(mMutexMap);
    mspKeyFrames.erase(pKF);

    // TODO: This only erase the pointer.
    // Delete the MapPoint
}

void Map::SetReferenceMapPoints(const vector<MapPoint *> &vpMPs)
{
    unique_lock<mutex> lock(mMutexMap);
    mvpReferenceMapPoints = vpMPs;
}

void Map::InformNewBigChange()
{
    unique_lock<mutex> lock(mMutexMap);
    mnBigChangeIdx++;
}

int Map::GetLastBigChangeIdx()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnBigChangeIdx;
}

vector<KeyFrame*> Map::GetAllKeyFrames()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<KeyFrame*>(mspKeyFrames.begin(),mspKeyFrames.end());
}

std::vector<MapAruco*> Map::GetAllMapArucos()
{
    unique_lock<mutex> lock(mMutexMap);
    // cout<<"in Map::GetAllMapArucos()"<<mspMapArucos.size()<<endl;
    return vector<MapAruco*>(mspMapArucos.begin(),mspMapArucos.end());
}

std::vector<int> Map::GetAllArucoMapPointID()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<int>(msAMPsid.begin(), msAMPsid.end());
}

vector<MapPoint*> Map::GetAllMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return vector<MapPoint*>(mspMapPoints.begin(),mspMapPoints.end());
}

long unsigned int Map::MapPointsInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspMapPoints.size();
}

long unsigned int Map::KeyFramesInMap()
{
    unique_lock<mutex> lock(mMutexMap);
    return mspKeyFrames.size();
}

vector<MapPoint*> Map::GetReferenceMapPoints()
{
    unique_lock<mutex> lock(mMutexMap);
    return mvpReferenceMapPoints;
}

long unsigned int Map::GetMaxKFid()
{
    unique_lock<mutex> lock(mMutexMap);
    return mnMaxKFid;
}

void Map::clear()
{
    for(set<MapPoint*>::iterator sit=mspMapPoints.begin(), send=mspMapPoints.end(); sit!=send; sit++)
        delete *sit;

    for(set<KeyFrame*>::iterator sit=mspKeyFrames.begin(), send=mspKeyFrames.end(); sit!=send; sit++)
        delete *sit;

    //! 应该包括MapAruco的删除
    for(set<MapAruco*>::iterator sit=mspMapArucos.begin(), send=mspMapArucos.end(); sit!=send; sit++)
        delete *sit;

    mspMapPoints.clear();
    mspKeyFrames.clear();
    mspMapArucos.clear();
    mnMaxKFid = 0;
    mvpReferenceMapPoints.clear();
    mvpKeyFrameOrigins.clear();
}

void Map::UpdateAruco()
{
    vector<int> vKFid;
    vector<KeyFrame*> vkf = this->GetAllKeyFrames();
    long unsigned int numkf = vkf.size();
    for(size_t i=0; i<numkf; i++)
    {
        KeyFrame* pk = vkf[i];
        int id = pk->mnId;
        vKFid.push_back(id);
    }
    for(set<MapAruco*>::iterator sit=mspMapArucos.begin(), send=mspMapArucos.end(); sit!=send; sit++)
    {
        MapAruco* pA = *sit;
        long unsigned int refkfid = pA->GetFirstKFid();
        for(long unsigned int i=0; i<numkf; i++) {
            if(vKFid[i] == refkfid) {
                pA->SetRtwmByKeyFrame(vkf[i]->GetRotation().t(), vkf[i]->GetCameraCenter());
                break;
            }
        }
    }
    // cout<<"Update Aruco"<<endl;
}

} //namespace ORB_SLAM
