/**
 * Add by liujiamin
 * TODO: to manage Aruco's corners
 */

#ifndef MAPARUCO_H
#define MAPARUCO_H

#include "Frame.h"
#include "KeyFrame.h"
#include "Map.h"
#include "MapPoint.h"
#include "Thirdparty/aruco/aruco/aruco.h"
#include <opencv2/core/core.hpp>
#include<mutex>

namespace ORB_SLAM2
{

class Frame;
class KeyFrame;
class Map;
class MapPoint;

// This class is like MapPoint
class MapAruco
{
public:
    MapAruco(const aruco::Marker& pM, KeyFrame* pRefKF, Map* pMap, double length);

    void    SetRtwm(const cv::Mat &Rwc, const cv::Mat &twc);
    void    SetRtwmByKeyFrame(const cv::Mat &Rwc, const cv::Mat &twc);
    cv::Mat GetTwm();
    void    SetPosInWorld();
    void    AddObservation(KeyFrame* pKF,size_t idx);
    int     GetMapArucoID();
    double  GetArucoLength();
    int     Observations();
    cv::Mat GetPosInWorld(const size_t & idx);
    MapPoint* GetMapPoint(const size_t & idx);
    // void    SetPosInWorld(const size_t & idx, const cv::Mat & p);
    std::map<KeyFrame*, size_t> GetObservations();
    static std::vector<cv::Point3f> get3DPointsLocalRefSystem(float length);
    aruco::Marker GetAruco();
    void SetCorrelateMapPoint(size_t idx, MapPoint* pMP);
    long unsigned int GetFirstKFid();

public:
    int nObs;
    long unsigned int mnFirstKFid;
    long int mnFirstFrame;
    bool mbAddLocalBA; // 这个应该又不需要了

    static std::mutex mGlobalMutex;

protected:
    aruco::Marker mAruco;
    cv::Mat mRcm;
    cv::Mat mtcm;

    cv::Mat mTwm; //获得Aruco相对世界坐标系的位置
    bool isTwm;
    cv::Mat mRwm;
    cv::Mat mtwm;
    double mLength;

    
    vector<cv::Mat> mvPosInTag;
    vector<cv::Mat> mvPosInWorld;
    vector<MapPoint*> mvMapPoint; //用于存储角点对应的mappoint

    // Keyframes observing the aruco and associated index in keyframe
    std::map<KeyFrame*,size_t> mObservations;

    std::mutex mMutexFeatures;
    std::mutex mMutexPos;
};

}


#endif // MAPARUCO_H