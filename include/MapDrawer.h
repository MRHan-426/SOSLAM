/**
 * @file MapDrawer.h
 * @author Raúl Mur-Artal, Yanmin Wu, thanks for your great work
 * @modified by Zhewei Ye
 * @Lastest modified on 19/04/2023
 */

#ifndef MAPDRAWER_H
#define MAPDRAWER_H

#include "SoSlam.h"
#include "Map.h"
#include <mutex>
#include <pangolin/pangolin.h>
#include <opencv2/core/core.hpp>

using namespace std;

namespace gtsam_soslam {
    class Map;
    class MapDrawer {
    public:
        MapDrawer(SoSlamState *sState, Map* mMap, const string &strSettingPath);

        SoSlamState *s;

        Map* mpMap;

        void DrawSemiDense(const double sigma);

        void DrawModel();

        void DrawTriangles(pangolin::OpenGlMatrix &Twc);

        void DrawFrame();

        bool drawPoints();

        void drawPointCloudLists(); // draw all the point cloud lists

        void drawPointCloudWithOptions(const std::map<std::string,bool> &options); // draw the point cloud lists with options opened

        void Coordinate();

        void DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph);

        // BRIEF [EAO-SLAM] draw objects.
        void DrawObject( const bool QuadricObj);

        void DrawGroundTruthObject();

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

} //namespace gtsam_soslam

#endif // MAPDRAWER_H
