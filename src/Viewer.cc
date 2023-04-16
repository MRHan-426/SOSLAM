/**
* This file is part of ORB-SLAM2.
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
* 
* Modification: EAO-SLAM
* Version: 1.0
* Created: 05/21/2019
* Author: Yanmin Wu
* E-mail: wuyanminmax@gmail.com
*/


#include <pangolin/pangolin.h>
//#include <pangolin/var/var.h>
//#include <pangolin/var/varextra.h>
//#include <pangolin/gl/gl.h>
//#include <pangolin/gl/gldraw.h>
//#include <pangolin/display/display.h>
//#include <pangolin/display/view.h>
//#include <pangolin/display/widgets.h>
//#include <pangolin/display/default_font.h>
//#include <pangolin/handler/handler.h>
#include "Viewer.h"
#include <mutex>

namespace gtsam_soslam {

    Viewer::Viewer(SoSlamState *sState, FrameDrawer *pFrameDrawer, MapDrawer *pMapDrawer,
                   const string &strSettingPath) :
            s(sState),
            mpFrameDrawer(pFrameDrawer), mpMapDrawer(pMapDrawer),
            mbFinishRequested(false), mbFinished(true), mbStopped(false), mbStopRequested(false) {
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        float fps = fSettings["Camera.fps"];
        if (fps < 1)
            fps = 30;
        mT = 1e3 / fps;

        mImageWidth = fSettings["Camera.width"];
        mImageHeight = fSettings["Camera.height"];
        if (mImageWidth < 1 || mImageHeight < 1) {
            mImageWidth = 640;
            mImageHeight = 480;
        }

        mViewpointX = fSettings["Viewer.ViewpointX"];
        mViewpointY = fSettings["Viewer.ViewpointY"];
        mViewpointZ = fSettings["Viewer.ViewpointZ"];
        mViewpointF = fSettings["Viewer.ViewpointF"];

        mfx = fSettings["Camera.fx"];
        mfy = fSettings["Camera.fy"];
        mcx = fSettings["Camera.cx"];
        mcy = fSettings["Camera.cy"];

    }

    void Viewer::Run() {
        mbFinished = false;

    // pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1024,768);
    pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",1600, 1080);   // 1920,1080.
    // pangolin::CreateWindowAndBind("ORB-SLAM2: Map Viewer",mImageWidth+175,mImageHeight);

        // 3D Mouse handler requires depth testing to be enabled
        glEnable(GL_DEPTH_TEST);

        // Issue specific OpenGl we might need
        glEnable(GL_BLEND);
        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

        pangolin::CreatePanel("menu").SetBounds(0.0, 1.0, 0.0, pangolin::Attach::Pix(175));
        pangolin::Var<bool> menuFollowCamera("menu.Follow Camera", true, true);
        pangolin::Var<bool> menuShowCamera("menu.Show Camera", true, true);
        pangolin::Var<bool> menuShowPoints("menu.Show Points", true, true);
        pangolin::Var<bool> menuShowKeyFrames("menu.Show KeyFrames", true, true);
        pangolin::Var<bool> menuShowGraph("menu.Show Graph", true, true);
        pangolin::Var<bool> menuShowSemiDense("menu.Show SemiDense", true, true);
        pangolin::Var<double> menuSigmaTH("menu.Sigma", 0.02, 1e-10, 0.05, false);
        pangolin::Var<bool> menuHandMove("menu.Move by Hand", false, true);
        pangolin::Var<bool> menuCameraView("menu.Camera View", true, true);
        pangolin::Var<bool> menuViewX("menu.X Axis View", false, true);
        pangolin::Var<bool> menuViewY("menu.Y Axis View", false, true);
        pangolin::Var<bool> menuViewZ("menu.Z Axis View", false, true);
//    pangolin::Var<bool> menuShowModel("menu.Show Model", false,true);
//    pangolin::Var<bool> menuShowTexture("menu.Show Texture", false,true);
        pangolin::Var<bool> menuShowAxis("menu.Show Axis", true, true);
        pangolin::Var<bool> menuShowGroundTruth("menu.Show Ground Truth", true, true);
//        pangolin::Var<bool> menuShowCubeObj("menu.Show CubeObj", true, true);
        pangolin::Var<bool> menuShowQuadricObj("menu.Show QuadricObj", true, true);

        pangolin::Var<bool> menuLocalizationMode("menu.Localization Mode", false, true);
        pangolin::Var<bool> menuReset("menu.Reset", false, false);



        // Define Camera Render Object (for view / scene browsing)
        pangolin::OpenGlRenderState s_cam(
//                pangolin::ProjectionMatrix(1024,768,mViewpointF,mViewpointF,512,389,0.1,1000),
//                pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0)
                // carv: using calibrated camera center and focal length
                pangolin::ProjectionMatrix(mImageWidth, mImageHeight, mfx, mfy, mcx, mcy, 0.1, 1000),
                pangolin::ModelViewLookAt(0, 0, 0, 0, 0, 1, 0.0, -1.0, 0.0)
        );

        // Add named OpenGL viewport to window and provide 3D Handler
        pangolin::View &d_cam = pangolin::CreateDisplay()
//            .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -1024.0f/768.0f)
                .SetBounds(0.0, 1.0, pangolin::Attach::Pix(175), 1.0, -mImageWidth / mImageHeight)
                .SetHandler(new pangolin::Handler3D(s_cam));

        pangolin::OpenGlMatrix Twc;
        Twc.SetIdentity();

        cv::namedWindow("Raw Image");
        cv::moveWindow("Raw Image", 40, 20);
        cv::namedWindow("Point, Line and Object Detection");
        cv::moveWindow("Point, Line and Object Detection", 40, 360);
        cv::namedWindow("Quadric Projection");
        cv::moveWindow("Quadric Projection", 40, 710);

        bool bFollow = true;
        bool bLocalizationMode = false;

        // carv: camera close up view
        bool bCameraView = false;
        pangolin::OpenGlMatrix projectionAbove = pangolin::ProjectionMatrix(mImageWidth, mImageHeight, mViewpointF,
                                                                            mViewpointF,
                                                                            mImageWidth / 2, mImageHeight / 2, 0.1,
                                                                            1000);
        pangolin::OpenGlMatrix projectionCamera = pangolin::ProjectionMatrix(mImageWidth, mImageHeight, mfx, mfy, mcx,
                                                                             mcy, 0.1, 1000);
        pangolin::OpenGlMatrix viewAbove = pangolin::ModelViewLookAt(mViewpointX, mViewpointY, mViewpointZ,
                                                                     0, 0, 0,
                                                                     0.0, -1.0, 0.0);
        pangolin::OpenGlMatrix viewCamera = pangolin::ModelViewLookAt(0, 0, 0,
                                                                      0, 0, 1,
                                                                      0.0, -1.0, 0.0);

        // Define the position of the camera
        gtsam::Point3 cameraPosition(0, 0, 0);

        // Define the orientations for the X, Y, and Z axes
        gtsam::Rot3 lookAtX = gtsam::Rot3::RzRyRx(-M_PI_2, 0, -M_PI_2); // Yaw, Pitch, Roll
        gtsam::Rot3 lookAtY = gtsam::Rot3::RzRyRx(-M_PI_2, 0, 0); // Yaw, Pitch, Roll
        gtsam::Rot3 lookAtZ = gtsam::Rot3::RzRyRx(M_PI, 0, 0); // Yaw, Pitch, Roll

        // Create gtsam::Pose3 instances for X, Y, and Z axes
        gtsam::Pose3 poseLookAtX(lookAtX, cameraPosition);
        gtsam::Pose3 poseLookAtY(lookAtY, cameraPosition);
        gtsam::Pose3 poseLookAtZ(lookAtZ, cameraPosition);
        pangolin::OpenGlMatrix viewFromX = mpMapDrawer->GetOpenGLCameraMatrixFromPose3(poseLookAtX);
        pangolin::OpenGlMatrix viewFromY = mpMapDrawer->GetOpenGLCameraMatrixFromPose3(poseLookAtY);
        pangolin::OpenGlMatrix viewFromZ = mpMapDrawer->GetOpenGLCameraMatrixFromPose3(poseLookAtZ);

        while (1) {
            glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

            mpMapDrawer->GetCurrentOpenGLCameraMatrix(Twc);

            if (menuFollowCamera && bFollow) {
                s_cam.Follow(Twc);
            } else if (menuFollowCamera && !bFollow) {
//            s_cam.SetModelViewMatrix(pangolin::ModelViewLookAt(mViewpointX,mViewpointY,mViewpointZ, 0,0,0,0.0,-1.0, 0.0));
                s_cam.Follow(Twc);
                bFollow = true;
            } else if (!menuFollowCamera && bFollow) {
                bFollow = false;

            }
            if(!bFollow){
                if(menuViewX && !menuViewY && !menuViewZ){
                    s_cam.Follow(viewFromX);
                } else if(!menuViewX && menuViewY && !menuViewZ){
                    s_cam.Follow(viewFromY);
                } else if(!menuViewX && !menuViewY && menuViewZ){
                    s_cam.Follow(viewFromZ);
                }
            }
            // if(menuLocalizationMode && !bLocalizationMode)
            // {
            //     mpSystem->ActivateLocalizationMode();
            //     bLocalizationMode = true;
            // }
            // else if(!menuLocalizationMode && bLocalizationMode)
            // {
            //     mpSystem->DeactivateLocalizationMode();
            //     bLocalizationMode = false;
            // }

            // carv: setup viewpoint to see model
            if(!menuHandMove)
                if (menuCameraView ) {
                    s_cam.SetProjectionMatrix(projectionCamera);
                    s_cam.SetModelViewMatrix(viewCamera);
                    bCameraView = true;
                } else  {

                        bCameraView = false;
//                    }
                }

            d_cam.Activate(s_cam);
            glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
            if (menuShowCamera)
                mpMapDrawer->DrawCurrentCamera(Twc);
             if(menuShowKeyFrames || menuShowGraph)
                mpMapDrawer->DrawKeyFrames(menuShowKeyFrames, menuShowGraph);
             if(menuShowPoints) {
                 mpMapDrawer->drawPoints();  // draw point clouds
                 // draw pointclouds with names
//                 RefreshPointCloudOptions();
//                 mpMapDrawer->drawPointCloudWithOptions(mmPointCloudOptionMap);
             }
            if(menuShowAxis)
                mpMapDrawer->Coordinate();
            // step draw objects.

            if(menuShowGroundTruth)
                mpMapDrawer->DrawGroundTruthObject();
            mpMapDrawer->DrawObject(menuShowQuadricObj);


            // TODO check opengl error?.
            //CheckGlDieOnError()
            // step carv: show model or triangle with light from camera
            // if(menuShowModel && menuShowTexture) {
            //     mpMapDrawer->DrawModel();
            // }
            // else if (menuShowModel && !menuShowTexture) {
            //     mpMapDrawer->DrawTriangles(Twc);
            // }
            // else if (!menuShowModel && menuShowTexture) {
            //     mpMapDrawer->DrawFrame();
            // }
            //CheckGlDieOnError()


            pangolin::FinishFrame();
// debug later
            // gray image.
            cv::Mat im = mpFrameDrawer->DrawFrame();
            if (!im.empty()) {
                cv::Mat resizeimg;
                cv::resize(im, resizeimg, cv::Size(640 * 0.7, 480 * 0.7), 0, 0, cv::INTER_CUBIC);
                cv::imshow("Point, Line and Object Detection", resizeimg);
            }

            // color image.
            cv::Mat RawImage = mpFrameDrawer->GetRawColorImage();
            if (!RawImage.empty()) {
                cv::Mat resizeimg;
                cv::resize(RawImage, resizeimg, cv::Size(640 * 0.7, 480 * 0.7), 0, 0, cv::INTER_CUBIC);
                cv::imshow("Raw Image", resizeimg);
            }

//         quadric image.
            cv::Mat QuadricImage = mpFrameDrawer->GetQuadricImage(menuShowQuadricObj, menuShowGroundTruth);
            if (!QuadricImage.empty()) {
            static int i=0;
            if(s->this_step.needSave()){
               cout<<"now i:"<<s->this_step.i<<endl;
                std::string filename = "../output/outputimage" + std::to_string(++i) + ".png";
                bool success = cv::imwrite(filename, QuadricImage);
                if (success) {
                    std::cout << "successfully store image" << std::endl;
                    s->this_step.imageSaved();
                }
            }
                cv::Mat resizeimg;
                cv::resize(QuadricImage, resizeimg, cv::Size(640 * 0.7, 480 * 0.7), 0, 0, cv::INTER_CUBIC);
                cv::imshow("Quadric Projection", resizeimg);
            }

            cv::waitKey(mT);

            if (menuReset) {
                menuShowGraph = true;
                menuShowKeyFrames = true;
                menuShowPoints = true;
                menuLocalizationMode = false;
                // if(bLocalizationMode)
                //     mpSystem->DeactivateLocalizationMode();
                bLocalizationMode = false;
                bFollow = true;
                menuFollowCamera = true;

                menuShowSemiDense = true;
                menuCameraView = true;
                menuViewX = false;
                menuViewY = false;
                menuViewZ = false;
                menuHandMove = false;
//            menuShowModel = true;
//            menuShowTexture = true;

                // mpSystem->Reset();
                menuReset = false;
                menuShowGroundTruth = true;
                menuShowAxis = true;
            }

            if (Stop()) {
                while (isStopped()) {
                    usleep(3000);
                }
            }

            if (CheckFinish())
                break;
        }

        SetFinish();
    }
//    void Viewer::RefreshPointCloudOptions()
//    {
//        // generate options from mmPointCloudOptionMenus, pointclouds with names will only be drawn when their options are activated.
//        std::map<std::string,bool> options;
//        for( auto pair : mmPointCloudOptionMenus)
//            options.insert(make_pair(pair.first, pair.second->Get()));
//
//        mmPointCloudOptionMap.clear();
//        mmPointCloudOptionMap = options;
//    }
    void Viewer::RefreshMenu(){
        unique_lock<mutex> lock(mMutexFinish);

        // Generate menu bar for every pointcloud in pointcloud list.
        auto pointLists = s->mpMap->GetPointCloudList();

        // Iterate over the menu and delete the menu if the corresponding clouds are no longer available
        for( auto menuPair = mmPointCloudOptionMenus.begin(); menuPair!=mmPointCloudOptionMenus.end(); )
        {
            if(pointLists.find(menuPair->first) == pointLists.end())
            {
                delete menuPair->second;        // destroy the dynamic menu
                menuPair = mmPointCloudOptionMenus.erase(menuPair);
            }
            else
                menuPair++;
        }

        // Iterate over the cloud lists to add new menu.
        for( auto cloudPair: pointLists )
        {
            if(mmPointCloudOptionMenus.find(cloudPair.first) == mmPointCloudOptionMenus.end())
            {
                pangolin::Var<bool>* pMenu = new pangolin::Var<bool>(string("menu.") + cloudPair.first, true, true);
                mmPointCloudOptionMenus.insert(make_pair(cloudPair.first, pMenu));
            }
        }

        // refresh double bars
        int doubleBarNum = mvDoubleMenus.size();
        int structNum = mvMenuStruct.size();
        if( structNum > 0 && structNum > doubleBarNum )
        {
            for(int i = doubleBarNum; i < structNum; i++)
            {
                pangolin::Var<double>* pMenu = new pangolin::Var<double>(string("menu.")+mvMenuStruct[i].name, mvMenuStruct[i].def, mvMenuStruct[i].min, mvMenuStruct[i].max);
                mvDoubleMenus.push_back(pMenu);
            }
        }

    }

    void Viewer::RequestFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinishRequested = true;
    }

    bool Viewer::CheckFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinishRequested;
    }

    void Viewer::SetFinish() {
        unique_lock<mutex> lock(mMutexFinish);
        mbFinished = true;
    }

    bool Viewer::isFinished() {
        unique_lock<mutex> lock(mMutexFinish);
        return mbFinished;
    }

    void Viewer::RequestStop() {
        unique_lock<mutex> lock(mMutexStop);
        if (!mbStopped)
            mbStopRequested = true;
    }

    bool Viewer::isStopped() {
        unique_lock<mutex> lock(mMutexStop);
        return mbStopped;
    }

    bool Viewer::Stop() {
        unique_lock<mutex> lock(mMutexStop);
        unique_lock<mutex> lock2(mMutexFinish);

        if (mbFinishRequested)
            return false;
        else if (mbStopRequested) {
            mbStopped = true;
            mbStopRequested = false;
            return true;
        }

        return false;

    }

    void Viewer::Release() {
        unique_lock<mutex> lock(mMutexStop);
        mbStopped = false;
    }

}
