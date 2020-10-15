/**
 * Add by liujiamin
 * TODO: to manage Aruco's corner
 */

#include "MapAruco.h"

namespace ORB_SLAM2
{

mutex MapAruco::mGlobalMutex;

MapAruco::MapAruco(const aruco::Marker& pM, KeyFrame* pRefKF, Map* pMap, double length):
nObs(0), mbAddLocalBA(false)
{
    cout<<"It's in the constuction..."<<endl;
    mAruco = pM; // 此mAruco会保存他在当前帧上的像素位置
    mLength = length;

    // TODO: Initial the position in the Tag Reference
    // (s/2,-s/2,0), (s/2,s/2,0), (-s/2,s/2,0), (-s/2,-s/2,0)
    cv::Mat c0=(cv::Mat_<float>(3,1)<< mLength/2, -mLength/2, 0);
    cv::Mat c1=(cv::Mat_<float>(3,1)<< mLength/2,  mLength/2, 0);
    cv::Mat c2=(cv::Mat_<float>(3,1)<<-mLength/2,  mLength/2, 0);
    cv::Mat c3=(cv::Mat_<float>(3,1)<<-mLength/2, -mLength/2, 0);
    mvPosInTag.push_back(c0);
    mvPosInTag.push_back(c1);
    mvPosInTag.push_back(c2);
    mvPosInTag.push_back(c3);

    // TODO: get rotation and transform from tag
    // 当一个新的Aruco出现的时候，先保存它与当前帧之间的R、t
    // 等当前帧位姿优化之后，再附上与世界坐标之间的关系，即 Rwm=Rwc*Rcm, twm=twc+tcm
    // 这样好像在 mAruco 里面已经包含了 Rvec, Tvec. 

    // TODO: Record FirstKFid | first Frame id
    mnFirstKFid = pRefKF->mnId;
    mnFirstFrame = pRefKF->mnFrameId;

}

void MapAruco::SetRtwm(const cv::Mat &Rwc, const cv::Mat &twc)
{
    unique_lock<mutex> lock(mMutexFeatures);
    cout<<"It's in the SetRTwm..."<<endl;
    cv::Mat rvec = mAruco.Rvec;
    cv::Mat Rcm = cv::Mat::zeros(3,3,CV_32F);
    cv::Rodrigues(rvec, Rcm);
    cout<<Rcm<<endl;
    mRwm = Rwc * Rcm;

    cv::Mat tcm = mAruco.Tvec;
    cout<<tcm<<endl;
    mtwm = twc + tcm;

    mTwm = cv::Mat::eye(4,4,CV_32F);
    mRwm.copyTo( mTwm.rowRange(0,3).colRange(0,3) );
    mtwm.copyTo( mTwm.rowRange(0,3).col(3) );

    SetPosInWorld();
}

cv::Mat MapAruco::GetTwm()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mTwm;
}

void MapAruco::SetPosInWorld()
{
    // unique_lock<mutex> lock(mMutexFeatures);
    cout<<"It's in the GetPosInWorld..."<<endl;
    mvPosInWorld.push_back(mRwm * mvPosInTag[0] + mtwm);
    mvPosInWorld.push_back(mRwm * mvPosInTag[1] + mtwm);
    mvPosInWorld.push_back(mRwm * mvPosInTag[2] + mtwm);
    mvPosInWorld.push_back(mRwm * mvPosInTag[3] + mtwm);
}

cv::Mat MapAruco::GetPosInWorld(const size_t & idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mvPosInWorld[idx];
}

void MapAruco::SetPosInWorld(const size_t & idx, const cv::Mat & p)
{
    unique_lock<mutex> lock(mMutexFeatures);
    mvPosInWorld[idx] = p;
}

void MapAruco::AddObservation(KeyFrame* pKF,size_t idx)
{
    unique_lock<mutex> lock(mMutexFeatures);
    cout<<"It's in the AddObservation..."<<endl;
    if(mObservations.count(pKF))
        return;
    mObservations[pKF]=idx;
    
    // pKF->mvuRightAruco[0/1/2/3] 的值都大于0，而且使用的是双目

    nObs+=2;
}

int MapAruco::GetMapArucoID()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mAruco.id;
}

int MapAruco::Observations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return nObs;
}

map<KeyFrame*, size_t> MapAruco::GetObservations()
{
    unique_lock<mutex> lock(mMutexFeatures);
    return mObservations;
}


}