/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* 
* Modification: EAO-SLAM
* Version: 1.0
* Created: 11/23/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

//#include"Map.h"
#include<opencv2/core/core.hpp>
// #include"MapPoint.h"
// #include"KeyFrame.h"
#include<pangolin/pangolin.h>

#include<mutex>
#include "SoSlam.h"
using namespace std;
namespace gtsam_soslam
{

class MapDrawer
{
public:
    MapDrawer(SoSlamState* sState, const string &strSettingPath);

    SoSlamState* s;

    void DrawSemiDense(const double sigma);
    void DrawModel();
    void DrawTriangles(pangolin::OpenGlMatrix &Twc);
    void DrawFrame();

    void DrawMapPoints();
    void Coordinate();
    void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

    // BRIEF [EAO-SLAM] draw objects.
    void DrawObject(const bool bCubeObj, const bool QuadricObj,
//                    const string &flag,
                    const bool bShowBottle,  const bool bShowChair, const bool bShowTvmonitors,
                    const bool bShowKeyboard,const bool bShowMouse, const bool bShowBook,   const bool bShowBear);

    void DrawCurrentCamera(pangolin::OpenGlMatrix &Twc);
    void SetCurrentCameraPose(const cv::Mat &Tcw);
    // void SetReferenceKeyFrame(KeyFrame *pKF);
    void GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M);
    pangolin::OpenGlMatrix GetOpenGLCameraMatrixFromPose3(gtsam::Pose3 &ps);

private:
    float mKeyFrameSize;
    float mKeyFrameLineWidth;
    float mGraphLineWidth;
    float mPointSize;
    float mCameraSize;
    float mCameraLineWidth;

    cv::Mat mCameraPose;

    std::mutex mMutexCamera;
};

} //namespace ORB_SLAM

#endif // MAPDRAWER_H
