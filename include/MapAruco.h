/**
 * Add by liujiamin
 * TODO: to manage Aruco's corners
 */

#ifndef MAPARUCO_H
#define MAPARUCO_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "Thirdparty/aruco/aruco/aruco.h"
#include <opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class Frame;
class KeyFrame;
class Map;

// This class is like MapPoint
class MapAruco
{
public:
    MapAruco(const aruco::Marker& pM, KeyFrame* pRefKF, Map* pMap, double length);

    void    SetRtwm(const cv::Mat &Rwc, const cv::Mat &twc);
    cv::Mat GetTwm();
    void    SetPosInWorld();
    void    AddObservation(KeyFrame* pKF,size_t idx);
    int     GetMapArucoID();
    int     Observations();
    cv::Mat GetPosInWorld(const size_t & idx);
    void    SetPosInWorld(const size_t & idx, const cv::Mat & p);
    std::map<KeyFrame*, size_t> GetObservations();

public:
    int nObs;
    long int mnFirstKFid;
    long int mnFirstFrame;
    bool mbAddLocalBA; // 这个应该又不需要了

    static std::mutex mGlobalMutex;

protected:
    //! important
    aruco::Marker mAruco;
    cv::Mat mTwm; //获得Aruco相对世界坐标系的位置
    cv::Mat mRwm;
    cv::Mat mtwm;
    double mLength;
    
    vector<cv::Mat> mvPosInTag;
    vector<cv::Mat> mvPosInWorld;

    // Keyframes observing the aruco and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;

    std::mutex mMutexFeatures;
};

}


#endif // MAPARUCO_H