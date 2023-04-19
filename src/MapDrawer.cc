/**
 * @file MapDrawer.cc
 * @author Raúl Mur-Artal, Yanmin Wu, thanks for your great work
 * @modified by Zhewei Ye
 * @Lastest modified on 19/04/2023
 */
#include "MapDrawer.h"

namespace gtsam_soslam {

    MapDrawer::MapDrawer(SoSlamState *sState, Map* mMap, const string &strSettingPath) :
    s(sState),
    mpMap(mMap){
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);

        mKeyFrameSize = fSettings["Viewer.KeyFrameSize"];
        mKeyFrameLineWidth = fSettings["Viewer.KeyFrameLineWidth"];
        mGraphLineWidth = fSettings["Viewer.GraphLineWidth"];
        mPointSize = fSettings["Viewer.PointSize"];
        mCameraSize = fSettings["Viewer.CameraSize"];
        mCameraLineWidth = fSettings["Viewer.CameraLineWidth"];
    }

    void MapDrawer::drawPointCloudLists()
    {
        auto pointLists = mpMap->GetPointCloudList();

        glPushMatrix();

        for(auto pair:pointLists){
            auto pPoints = pair.second;
            if( pPoints == NULL ) continue;
            for(int i=0; i<pPoints->size(); i=i+1)
            {
                PointXYZRGB &p = (*pPoints)[i];
                glPointSize( p.size );
                glBegin(GL_POINTS);
                glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
                glVertex3d(p.x, p.y, p.z);
                glEnd();

            }
        }
        glPointSize( 1 );

        glPopMatrix();
    }

    bool MapDrawer::drawPoints() {
        vector<PointXYZRGB*> pPoints = mpMap->GetAllPoints();
        glPushMatrix();

        for(int i=0; i<pPoints.size(); i=i+1)
        {
            PointXYZRGB &p = *(pPoints[i]);
            glPointSize( p.size );
            glBegin(GL_POINTS);
            glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
            glVertex3d(p.x, p.y, p.z);
            glEnd();
        }

        glPointSize( 1 );
        glPopMatrix();

        return true;
    }

    void MapDrawer::drawPointCloudWithOptions(const std::map<std::string,bool> &options)
    {
        auto pointLists = mpMap->GetPointCloudList();
        glPushMatrix();

        for(auto pair:pointLists){
            auto pPoints = pair.second;
            if( pPoints == NULL ) continue;

            auto iter = options.find(pair.first);
            if(iter == options.end()) {
                continue;  // not exist
            }
            if(iter->second == false) continue; // menu is closed

            for(int i=0; i<pPoints->size(); i=i+1)
            {
                PointXYZRGB &p = (*pPoints)[i];
                glPointSize( p.size );
                glBegin(GL_POINTS);
                glColor3d(p.r/255.0, p.g/255.0, p.b/255.0);
                glVertex3d(p.x, p.y, p.z);
                glEnd();

            }
        }
        glPointSize( 1 );
        glPopMatrix();
    }

//void MapDrawer::DrawFrame()
//{
//    const vector<KeyFrame*> &vpKf = mpMap->GetAllKeyFrames();
//    if(vpKf.empty()) return;
//
//    // get the most recent reconstructed keyframe to texture
//    KeyFrame* kfToTexture = NULL;
//    KeyFrame* prevKf = NULL;
//    for(size_t i = 0; i < vpKf.size();++i) {
//        KeyFrame *kf = vpKf[i];
//        kf->SetNotEraseDrawer();
//        if (kf->isBad() || !kf->semidense_flag_ || !kf->interKF_depth_flag_) {
//            kf->SetEraseDrawer();
//            continue;
//        }
//        if (prevKf == NULL){
//            kfToTexture = kf;
//            prevKf = kf;
//        } else if (kf->mnId > prevKf->mnId){
//            kfToTexture = kf;
//            prevKf->SetEraseDrawer();
//            prevKf = kf;
//        }
//    }
//    if (kfToTexture == NULL) return;
//
//
//    cv::Size imSize = kfToTexture->rgb_.size();
//
//    pangolin::GlTexture imageTexture(imSize.width, imSize.height, GL_RGB, false, 0, GL_BGR,
//                                     GL_UNSIGNED_BYTE);
//
//    imageTexture.Upload(kfToTexture->rgb_.data, GL_BGR, GL_UNSIGNED_BYTE);
//
//    imageTexture.RenderToViewportFlipY();
//
//
//    kfToTexture->SetEraseDrawer();
//}

    void MapDrawer::Coordinate() {
        // draw xyz frame
        glLineWidth(3);
        glBegin(GL_LINES);
        glColor3f(0.8f, 0.f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0.5, 0, 0);
        glColor3f(0.f, 0.8f, 0.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0.5, 0);
        glColor3f(0.2f, 0.2f, 1.f);
        glVertex3f(0, 0, 0);
        glVertex3f(0, 0, 0.5);
        glEnd();
    }

    void MapDrawer::DrawKeyFrames(const bool bDrawKF, const bool bDrawGraph) {
        const float &w = mKeyFrameSize;
        const float h = w * 0.75;
        const float z = w * 0.6;
        if(bDrawKF) {
            const gtsam::Values vEstimates = s->estimates_;
            auto current_ps_qs = utils::ps_and_qs_from_values(vEstimates);
            std::map<gtsam::Key, gtsam::Pose3> cps = current_ps_qs.first;
//     const vector<KeyFrame*> vpKFs = mpMap->GetAllKeyFrames();

//     if(bDrawKF)
//     {
//    cout<<"previous poses"<<endl;
//    int i=-1;
            for (auto &ps: cps) {
//         for(size_t i=0; i<vpKFs.size(); i++)
//         {
//            i++;
                gtsam::Pose3 p = ps.second;
//             KeyFrame* pKF = vpKFs[i];
                pangolin::OpenGlMatrix Tw = GetOpenGLCameraMatrixFromPose3(p);
//             cv::Mat Twc = pKF->GetPoseInverse().t();
//            cout<< p.matrix()<<endl;
                glPushMatrix();

//             glMultMatrixf(Twc.m);
                glMultMatrixd(Tw.m);

                glLineWidth(mKeyFrameLineWidth);

                // [EAO] created by objects.
//             if(pKF->mbCreatedByObjs)
//                 glColor3f(1.0f,0.0f,0.0f);
//             else
                glColor3f(0.0f, 0.0f, 1.0f);

                glBegin(GL_LINES);
                glVertex3f(0, 0, 0);
                glVertex3f(w, h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, -h, z);
                glVertex3f(0, 0, 0);
                glVertex3f(-w, h, z);

                glVertex3f(w, h, z);
                glVertex3f(w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(-w, -h, z);

                glVertex3f(-w, h, z);
                glVertex3f(w, h, z);

                glVertex3f(-w, -h, z);
                glVertex3f(w, -h, z);
                glEnd();

                glPopMatrix();
            }

    //         cout<<"number of pose "<<i<<endl;
    //     }

    //     if(bDrawGraph)
    //     {
            glLineWidth(mGraphLineWidth);
            glColor4f(0.0f, 1.0f, 0.0f, 0.6f);
            glBegin(GL_LINES);
            pair<gtsam::Key, gtsam::Pose3> previous = {0, gtsam::Pose3::identity()};
            pangolin::OpenGlMatrix PrevM;
            PrevM.SetIdentity();
            pair<gtsam::Key, gtsam::Pose3> current = {0, gtsam::Pose3::identity()};
            pangolin::OpenGlMatrix CurM;
            CurM.SetIdentity();
            for (auto &ps: cps) {
                previous = current;
                PrevM = CurM;
                current = ps;
                CurM = GetOpenGLCameraMatrixFromPose3(ps.second);
                if (previous.first) {
    //                 M.m[12] = poseMatrix(0,3);//twc.at<float>(0);
    //                 M.m[13] = poseMatrix(1,3);//twc.at<float>(1);
    //                 M.m[14] = poseMatrix(2,3);//twc.at<float>(2);
    //                 cv::Mat Ow2 = (*vit)->GetCameraCenter();
                    glVertex3f(PrevM.m[12], PrevM.m[13], PrevM.m[14]);
                    glVertex3f(CurM.m[12], CurM.m[13], CurM.m[14]);
                }
    //         for(size_t i=0; i<vpKFs.size(); i++)
    //         {
                // Covisibility Graph
    //             const vector<KeyFrame*> vCovKFs = vpKFs[i]->GetCovisiblesByWeight(100);
    //             cv::Mat Ow = vpKFs[i]->GetCameraCenter();
    //             if(!vCovKFs.empty())
    //             {
    //                 for(vector<KeyFrame*>::const_iterator vit=vCovKFs.begin(), vend=vCovKFs.end(); vit!=vend; vit++)
    //                 {
    //                     if((*vit)->mnId<vpKFs[i]->mnId)
    //                         continue;
    //                     cv::Mat Ow2 = (*vit)->GetCameraCenter();
    //                     glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //                     glVertex3f(Ow2.at<float>(0),Ow2.at<float>(1),Ow2.at<float>(2));
    //                 }
    //             }
    //
    //             // Spanning tree
    //             KeyFrame* pParent = vpKFs[i]->GetParent();
    //             if(pParent)
    //             {
    //                 cv::Mat Owp = pParent->GetCameraCenter();
    //                 glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //                 glVertex3f(Owp.at<float>(0),Owp.at<float>(1),Owp.at<float>(2));
    //             }
    //
    //             // Loops
    //             set<KeyFrame*> sLoopKFs = vpKFs[i]->GetLoopEdges();
    //             for(set<KeyFrame*>::iterator sit=sLoopKFs.begin(), send=sLoopKFs.end(); sit!=send; sit++)
    //             {
    //                 if((*sit)->mnId<vpKFs[i]->mnId)
    //                     continue;
    //                 cv::Mat Owl = (*sit)->GetCameraCenter();
    //                 glVertex3f(Ow.at<float>(0),Ow.at<float>(1),Ow.at<float>(2));
    //                 glVertex3f(Owl.at<float>(0),Owl.at<float>(1),Owl.at<float>(2));
    //             }
            }

            glEnd();
        }
//     }
    }

// BRIEF [EAO-SLAM] draw objects.
    void MapDrawer::DrawObject(const bool QuadricObj) {
//    const vector<Object_Map*> &vObjs = mpMap->GetObjects();
    if(QuadricObj) {
        const gtsam::Values vEstimates = s->estimates_;
        auto current_ps_qs = utils::ps_and_qs_from_values(vEstimates);
        std::map<gtsam::Key, ConstrainedDualQuadric> cqs = current_ps_qs.second;
        vector<cv::Mat> object_cen;

        int i = -1;
    //        std::vector<ConstrainedDualQuadric> qs = Constants::QUADRICS;
        for (auto &key_value: cqs) {
            i++;
            gtsam::Key ObjKey = key_value.first;
            ConstrainedDualQuadric *Obj = &(key_value.second);

            // color.
            if (i % 10 == 0)
                glColor3f(0.0 / 255.0, 0.0 / 255.0, 135.0 / 255.0);
            else if (i % 10 == 1)
                glColor3f(135.0 / 255.0, 0.0 / 255.0, 135.0 / 255.0);
            else if (i % 10 == 2)
                glColor3f(119.0 / 255.0, 254.0 / 255.0, 4.0 / 255.0);
            else if (i % 10 == 3)
                glColor3f(1.0 / 255.0, 126.0 / 255.0, 255.0 / 255.0);
            else if (i % 10 == 4)
                glColor3f(255.0 / 255.0, 112.0 / 255.0, 0.0 / 255.0);
            else if (i % 10 == 5)
                glColor3f(250.0 / 255.0, 250.0 / 255.0, 0.0 / 255.0);
            else if (i % 10 == 6)
                glColor3f(1.0, 1.0, 0);
            else if (i % 10 == 7)
                glColor3f(1.0, 0, 1.0);
            else if (i % 10 == 8)
                glColor3f(0.5, 0.5, 0.0);
            else if (i % 10 == 9)
                glColor3f(0.5, 0, 0.5);

            glLineWidth(mCameraLineWidth);

            // ****************************************
            //    STEP 2. [EAO-SLAM] Draw quadrics.   *
            // ****************************************

            // half axial length.
            gtsam::Vector3 radii = Obj->radii();
            double lenth = radii[0];
            double width = radii[1];
            double height = radii[2];


            cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
            axe.at<float>(0) = lenth;
            axe.at<float>(1) = width;
            axe.at<float>(2) = height;

            gtsam::Point3 centroid = Obj->centroid();
            // quadrcis pose.
            ;
    //        cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
    //        cv::Mat Twq(4, 4, CV_32FC1, Obj->pose().matrix().data());
            gtsam::Pose3 pose = Obj->pose();
            gtsam::Rot3 rot = pose.rotation();
            gtsam::Matrix33 rot_matrix = rot.matrix();

            gtsam::Matrix44 homogeneous_matrix = pose.matrix();
            std::vector<GLfloat> gl_matrix(homogeneous_matrix.data(),
                                           homogeneous_matrix.data() + homogeneous_matrix.size());

            // create a quadric.
            GLUquadricObj *pObj = gluNewQuadric();

            // color
            cv::Scalar sc;
            sc = cv::Scalar(0, 255, 0);

            // add to display list
            glPushMatrix();
    //        glMultMatrixf(Twq_t.ptr<GLfloat >(0));
            glMultMatrixf(gl_matrix.data());

            glScalef(
                    (GLfloat) (axe.at<float>(0, 0)),
                    (GLfloat) (axe.at<float>(0, 1)),
                    (GLfloat) (axe.at<float>(0, 2))
            );

            gluQuadricDrawStyle(pObj, GLU_LINE);
            gluQuadricNormals(pObj, GLU_NONE);
            glBegin(GL_COMPILE);
            gluSphere(pObj, 1., 15, 10);

            glEnd();
            glPopMatrix();
            // draw quadrics END ---------------------------------------------------------------------
    //        }
        }
    }

    } // draw objects END ----------------------------------------------------------------------------
    void MapDrawer::DrawGroundTruthObject()
    {
        std::vector<ConstrainedDualQuadric> groundTruthQuad = Constants::groundTruthQuadrics();
        vector<cv::Mat> object_cen;

        int i = -1;
        for (auto &Obj: groundTruthQuad) {
            i++;
            glColor3f(1, 0, 0);

            glLineWidth(mCameraLineWidth);

            // ****************************************
            //    STEP 2. [EAO-SLAM] Draw quadrics.   *
            // ****************************************
//        if(QuadricObj && !((Obj->mnClass == 73) || (Obj->mnClass == 64) || (Obj->mnClass == 65)
//                || (Obj->mnClass == 66) || (Obj->mnClass == 56) || (Obj->mnClass == 72)))
//        {
            // half axial length.
            gtsam::Vector3 radii = Obj.radii();
            double lenth = radii[0];
            double width = radii[1];
            double height = radii[2];


            cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
            axe.at<float>(0) = lenth;
            axe.at<float>(1) = width;
            axe.at<float>(2) = height;

            gtsam::Point3 centroid = Obj.centroid();
            // quadrcis pose.
            ;
//        cv::Mat Twq = cv::Mat::zeros(4,4,CV_32F);
//        cv::Mat Twq(4, 4, CV_32FC1, Obj->pose().matrix().data());
            gtsam::Pose3 pose = Obj.pose();
            gtsam::Rot3 rot = pose.rotation();
            gtsam::Matrix33 rot_matrix = rot.matrix();

//            gtsam::Matrix44 homogeneous_matrix = gtsam::Matrix44::Identity();
//            homogeneous_matrix.block<3, 3>(0, 0) = rot_matrix;
            gtsam::Matrix44 homogeneous_matrix = pose.matrix();
            std::vector<GLfloat> gl_matrix(homogeneous_matrix.data(),
                                           homogeneous_matrix.data() + homogeneous_matrix.size());

            // create a quadric.
            GLUquadricObj *pObj = gluNewQuadric();
//        cv::Mat Twq_t = Twq;

            // color
            cv::Scalar sc;
            sc = cv::Scalar(0, 255, 0);

            // add to display list
            glPushMatrix();
//        glMultMatrixf(Twq_t.ptr<GLfloat >(0));
            glMultMatrixf(gl_matrix.data());

            glScalef(
                    (GLfloat) (axe.at<float>(0, 0)),
                    (GLfloat) (axe.at<float>(0, 1)),
                    (GLfloat) (axe.at<float>(0, 2))
            );

            gluQuadricDrawStyle(pObj, GLU_LINE);
            gluQuadricNormals(pObj, GLU_NONE);
            glBegin(GL_COMPILE);
            gluSphere(pObj, 1., 15, 10);

            glEnd();
            glPopMatrix();
            // draw quadrics END ---------------------------------------------------------------------
//        }
        }

    } // draw objects END ----------------------------------------------------------------------------

    void MapDrawer::DrawCurrentCamera(pangolin::OpenGlMatrix &Twc) {

        const float &w = mCameraSize;
        const float h = w * 0.75;
        const float z = w * 0.6;

        glPushMatrix();

#ifdef HAVE_GLES
        glMultMatrixf(Twc.m);
#else
        glMultMatrixd(Twc.m);
#endif

        glLineWidth(mCameraLineWidth);
        glColor3f(0.0f, 1.0f, 0.0f);
        glBegin(GL_LINES);
        glVertex3f(0, 0, 0);
        glVertex3f(w, h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, -h, z);
        glVertex3f(0, 0, 0);
        glVertex3f(-w, h, z);

        glVertex3f(w, h, z);
        glVertex3f(w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(-w, -h, z);

        glVertex3f(-w, h, z);
        glVertex3f(w, h, z);

        glVertex3f(-w, -h, z);
        glVertex3f(w, -h, z);
        glEnd();

        glPopMatrix();
    }


    void MapDrawer::SetCurrentCameraPose(const cv::Mat &Tcw) {
        unique_lock<mutex> lock(mMutexCamera);
        mCameraPose = Tcw.clone();
    }

    void MapDrawer::GetCurrentOpenGLCameraMatrix(pangolin::OpenGlMatrix &M) {
//    if(!mCameraPose.empty())
//    {
//    cout<<"current camera"<<endl;
        cv::Mat Rwc(3, 3, CV_32F);
        cv::Mat twc(3, 1, CV_32F);
//        {
        gtsam::Pose3 pose3 = s->this_step.odom;
        gtsam::Matrix44 poseMatrix = pose3.matrix();
//        unique_lock<mutex> lock(mMutexCamera);
//        Rwc = mCameraPose.rowRange(0,3).colRange(0,3).t();
//        twc = -Rwc*mCameraPose.rowRange(0,3).col(3);
//        }
//        cout<< poseMatrix<<endl;
        M.m[1] = poseMatrix(1, 0);//Rwc.at<float>(1,0);
        M.m[0] = poseMatrix(0, 0);//Rwc.at<float>(0,0);
        M.m[2] = poseMatrix(2, 0);//Rwc.at<float>(2,0);
        M.m[3] = 0.0;

        M.m[4] = poseMatrix(0, 1);//Rwc.at<float>(0,1);
        M.m[5] = poseMatrix(1, 1);//Rwc.at<float>(1,1);
        M.m[6] = poseMatrix(2, 1);//Rwc.at<float>(2,1);
        M.m[7] = 0.0;

        M.m[8] = poseMatrix(0, 2);//Rwc.at<float>(0,2);
        M.m[9] = poseMatrix(1, 2);//Rwc.at<float>(1,2);
        M.m[10] = poseMatrix(2, 2);//Rwc.at<float>(2,2);
        M.m[11] = 0.0;

        M.m[12] = poseMatrix(0, 3);//twc.at<float>(0);
        M.m[13] = poseMatrix(1, 3);//twc.at<float>(1);
        M.m[14] = poseMatrix(2, 3);//twc.at<float>(2);
        M.m[15] = 1.0;
//    }
//    else
//        M.SetIdentity();
    }

    pangolin::OpenGlMatrix MapDrawer::GetOpenGLCameraMatrixFromPose3(gtsam::Pose3 &ps) {
        pangolin::OpenGlMatrix M;
        M.SetIdentity();
//        cv::Mat Rwc(3, 3, CV_32F);
//        cv::Mat twc(3, 1, CV_32F);

//        gtsam::Pose3 pose3=ps;
        gtsam::Matrix44 poseMatrix = ps.matrix();

        M.m[0] = poseMatrix(0, 0);//Rwc.at<float>(0,0);
        M.m[1] = poseMatrix(1, 0);//Rwc.at<float>(1,0);
        M.m[2] = poseMatrix(2, 0);//Rwc.at<float>(2,0);
        M.m[3] = 0.0;

        M.m[4] = poseMatrix(0, 1);//Rwc.at<float>(0,1);
        M.m[5] = poseMatrix(1, 1);//Rwc.at<float>(1,1);
        M.m[6] = poseMatrix(2, 1);//Rwc.at<float>(2,1);
        M.m[7] = 0.0;

        M.m[8] = poseMatrix(0, 2);//Rwc.at<float>(0,2);
        M.m[9] = poseMatrix(1, 2);//Rwc.at<float>(1,2);
        M.m[10] = poseMatrix(2, 2);//Rwc.at<float>(2,2);
        M.m[11] = 0.0;

        M.m[12] = poseMatrix(0, 3);//twc.at<float>(0);
        M.m[13] = poseMatrix(1, 3);//twc.at<float>(1);
        M.m[14] = poseMatrix(2, 3);//twc.at<float>(2);
        M.m[15] = 1.0;
        return M;
    }

} //namespace ORB_SLAM
