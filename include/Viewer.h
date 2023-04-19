/**
 * @file MapDrawer.cc
 * @author Raúl Mur-Artal, thanks for your great work
 * @modified by Zhewei Ye
 * @Lastest modified on 19/04/2023
 */

#ifndef VIEWER_H
#define VIEWER_H

#include "SoSlam.h"
#include "FrameDrawer.h"
#include "MapDrawer.h"
#include <unistd.h>
#include <mutex>
using namespace std;

namespace gtsam_soslam {
    struct MenuStruct
    {
        double min;
        double max;
        double def;
        string name;
    };
// class Tracking;
    class FrameDrawer;

    class MapDrawer;
// class System;

    class Viewer {
    public:
        Viewer(SoSlamState *sState, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
               const string &strSettingPath);

        // Main thread function. Draw points, keyframes, the current camera pose and the last processed
        // frame. Drawing is refreshed according to the camera fps. We use Pangolin.
        void Run();

        void RequestFinish();

        void RequestStop();

        bool isFinished();

        bool isStopped();

        void Release();

    private:

        bool Stop();

        // System* mpSystem;
        SoSlamState *s;
        FrameDrawer *mpFrameDrawer;
        MapDrawer *mpMapDrawer;
        // Tracking* mpTracker;

        // 1/fps in ms
        double mT;
        float mImageWidth, mImageHeight;

        float mViewpointX, mViewpointY, mViewpointZ, mViewpointF;
        float mfx, mfy, mcx, mcy;

        bool CheckFinish();

        void SetFinish();

        bool mbFinishRequested;
        bool mbFinished;
        std::mutex mMutexFinish;

        bool mbStopped;
        bool mbStopRequested;
        std::mutex mMutexStop;

        // menu lists
        vector<pangolin::Var<double>*> mvDoubleMenus;
        vector<MenuStruct> mvMenuStruct;

        // Manual control of the point cloud visualization
        map<string,pangolin::Var<bool>*> mmPointCloudOptionMenus;
        void RefreshPointCloudOptions();

        void RefreshMenu();
        // demo.
//    string mflag;
    };

}

#endif // VIEWER_H
	

