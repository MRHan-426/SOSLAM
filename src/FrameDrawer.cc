/**
 * @file FrameDrawer.cc
 * @author Raúl Mur-Artal, Yanmin Wu, thanks for your great work
 * @modified by Zhewei Ye
 * @Lastest modified on 19/04/2023
 */

#include "FrameDrawer.h"
#include <opencv2/core/core.hpp>
#include <mutex>

namespace gtsam_soslam {

    FrameDrawer::FrameDrawer(SoSlamState *sState) : s(sState) {
        // mState=Tracking::SYSTEM_NOT_READY;
        mIm = cv::Mat(480, 640, CV_8UC3, cv::Scalar(0, 0, 0));
    }

    cv::Mat FrameDrawer::DrawFrame() {
        cv::Mat im;
        vector<cv::KeyPoint> vIniKeys; // Initialization: KeyPoints in reference frame
        vector<int> vMatches; // Initialization: correspondeces with reference keypoints
        vector<cv::KeyPoint> vCurrentKeys; // KeyPoints in current frame
        vector<bool> vbVO, vbMap; // Tracked MapPoints in current frame
        if (!s->this_step.rgb.empty())
            s->this_step.rgb.copyTo(mRGBIm);         // [EAO] copy color image.
        else
            mIm.copyTo(mRGBIm);
        Dboxes = s->this_step.detections;
        // yolo detection.
        mRGBIm.copyTo(mDetIm);
        DrawYoloInfo(mDetIm, true);
        //return imWithInfo;
        return mDetIm;
    }

    // BRIEF [EAO-SLAM] draw yolo image.
    cv::Mat FrameDrawer::DrawYoloFrame() {
        cv::Mat imRGB;
        mRGBIm.copyTo(imRGB);
        DrawYoloInfo(imRGB, true);
        return imRGB;
    }

    // BRIEF [EAO-SLAM] get color image.
    cv::Mat FrameDrawer::GetRawColorImage() {
        cv::Mat imRGB;
        mRGBIm.copyTo(imRGB);
        return imRGB;
    }

    // BRIEF [EAO-SLAM] draw quadric image.
    cv::Mat FrameDrawer::GetQuadricImage(const bool menuShowQuadricObj, const bool menuShowGroundTruth) {
        //    cv::Mat imRGB;
        mRGBIm.copyTo(mQuadricIm);
        //    mQuadricIm.copyTo(imRGB);
        const gtsam::Values vEstimates = s->estimates_;
        auto current_ps_qs = utils::ps_and_qs_from_values(vEstimates);
        std::map<gtsam::Key, ConstrainedDualQuadric> cqs = current_ps_qs.second;
        vector<cv::Mat> object_cen;

        gtsam::Matrix3 K = s->calib_rgb_.K();
        gtsam::Matrix4 Xi = s->this_step.odom.inverse().matrix();
        static gtsam::Matrix34 I34 = gtsam::Matrix::Identity(3, 4);
        gtsam::Matrix34 P = K * I34 * Xi;
        cv::Mat cv_mat(3, 4, CV_32FC1); // Create a 3x4 OpenCV matrix with double precision elements

        // Copy the data from the gtsam::Matrix34 object to the cv::Mat object
        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 4; ++j) {
                cv_mat.at<float>(i, j) = P(i, j);
            }
        }
        if(menuShowGroundTruth) {
            std::vector<ConstrainedDualQuadric> groundTruthQuad = s->groundTruthes[s->dataset];
            for (auto &Obj: groundTruthQuad) {
                gtsam::Vector3 radii = Obj.radii();
                double lenth = radii[0];
                double width = radii[1];
                double height = radii[2];

                // step 10.7 project quadrics to the image (only for visualization).
                cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
                axe.at<float>(0) = lenth;
                axe.at<float>(1) = width;
                axe.at<float>(2) = height;

                gtsam::Pose3 pose = Obj.pose(); // assume pose is initialized
                cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);
                cv::Mat R = cv::Mat::eye(4, 4, CV_32FC1);

                // set translation matrix
                T.at<float>(0, 3) = pose.translation().x();
                T.at<float>(1, 3) = pose.translation().y();
                T.at<float>(2, 3) = pose.translation().z();

                // set rotation matrix
                gtsam::Rot3 rot = pose.rotation();
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        R.at<float>(i, j) = rot.matrix()(i, j);
                    }
                }

                // concatenate rotation and translation matrices
                cv::Mat Twq = T * R;
                // create a quadric.
                cv::Mat Twq_t = Twq.t();

                mQuadricIm = DrawQuadricProject(mQuadricIm,
                                                cv_mat,
                                                axe,
                                                Twq,
                                                -1);
            }
        }
        int i = -1;
        //    std::vector<ConstrainedDualQuadric> qs = Constants::QUADRICS;
        if(menuShowQuadricObj) {
            for (auto &key_value: cqs) {
                i++;
                gtsam::Key ObjKey = key_value.first;
                ConstrainedDualQuadric *Obj = &(key_value.second);
                bool pass=true;
                auto dets = s->this_step.detections;
                for(auto &det : dets){
                    if(ObjKey == det.quadric_key)
                        pass=false;
                }
                if(pass)
                    continue;
                //    for (auto& q : qs) {
                //        i++;
                //        gtsam::Key ObjKey = key_value.first;
                //        ConstrainedDualQuadric *Obj = &(q);
                gtsam::Vector3 radii = Obj->radii();
                double lenth = radii[0];
                double width = radii[1];
                double height = radii[2];

                // step 10.7 project quadrics to the image (only for visualization).

                cv::Mat axe = cv::Mat::zeros(3, 1, CV_32F);
                axe.at<float>(0) = lenth;
                axe.at<float>(1) = width;
                axe.at<float>(2) = height;

                gtsam::Point3 centroid = Obj->centroid();
                // quadrcis pose.
                // object pose (world).
                gtsam::Pose3 pose = Obj->pose(); // assume pose is initialized
                cv::Mat T = cv::Mat::eye(4, 4, CV_32FC1);
                cv::Mat R = cv::Mat::eye(4, 4, CV_32FC1);

                // set translation matrix
                T.at<float>(0, 3) = pose.translation().x();
                T.at<float>(1, 3) = pose.translation().y();
                T.at<float>(2, 3) = pose.translation().z();

                // set rotation matrix
                gtsam::Rot3 rot = pose.rotation();
                for (int i = 0; i < 3; i++) {
                    for (int j = 0; j < 3; j++) {
                        R.at<float>(i, j) = rot.matrix()(i, j);
                    }
                }

                // concatenate rotation and translation matrices
                cv::Mat Twq = T * R;
                // create a quadric.
                cv::Mat Twq_t = Twq.t();

                mQuadricIm = DrawQuadricProject(mQuadricIm,
                                                cv_mat,
                                                axe,
                                                Twq,
                                                i);
            }
        }

        return mQuadricIm;
    }

    // BRIEF draw yolo text.
    cv::Mat FrameDrawer::DrawYoloInfo(cv::Mat &im, bool bText) {
        for (auto &box: Dboxes) {
            double x1 = box.bounds[0];
            double y1 = box.bounds[1];
            double x2 = box.bounds[2];
            double y2 = box.bounds[3];
            cv::Point2d tl(x1, y1);
            cv::Point2d br(x2, y2);

            if (bText) {
                cv::putText(im,
                            box.label,
                            tl,
                            cv::FONT_HERSHEY_DUPLEX,
                            1.0,
                            colors[0],
                        // cv::Scalar(0,255,0),
                            2);
            }

            // draw bounding box.
            cv::rectangle(im,
                          tl,
                          br,
                          colors[0],
                          2);
        }

        return im;
    } // DrawYoloInfo().

    void FrameDrawer::Update() {
        unique_lock<mutex> lock(mMutex);
        s->this_step.rgb.copyTo(mRGBIm);         // [EAO] copy color image.
        Dboxes = s->this_step.detections;}

    // BRIEF [EAO] project quadrics from world to image.
    cv::Mat FrameDrawer::DrawQuadricProject(cv::Mat &im,
                                            const cv::Mat &P,   // projection matrix.
                                            const cv::Mat &axe, // axis length.
                                            const cv::Mat &Twq, // object pose.
                                            int nClassid,
                                            bool isGT,
                                            int nLatitudeNum,
                                            int nLongitudeNum) {
        // color.
        std::vector<cv::Scalar> colors = {cv::Scalar(135, 0, 0),
                                          cv::Scalar(135, 0, 135),
                                          cv::Scalar(4, 254, 119),
                                          cv::Scalar(255, 126, 1),
                                          cv::Scalar(0, 112, 255),
                                          cv::Scalar(0, 250, 250),
        };

        // draw params
        cv::Scalar sc;
        if(nClassid==-1)
            sc = cv::Scalar(0, 0, 255);
        else
            sc  = colors[nClassid % 6];

        int nLineWidth = 2;

        // generate angluar grid -> xyz grid (vertical half sphere)
        vector<float> vfAngularLatitude;  // (-90, 90)
        vector<float> vfAngularLongitude; // [0, 180]
        cv::Mat pointGrid(nLatitudeNum + 2, nLongitudeNum + 1, CV_32FC4);

        for (int i = 0; i < nLatitudeNum + 2; i++) {
            float fThetaLatitude = -M_PI_2 + i * M_PI / (nLatitudeNum + 1);
            cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
            for (int j = 0; j < nLongitudeNum + 1; j++) {
                float fThetaLongitude = j * M_PI / nLongitudeNum;
                p[j][0] = axe.at<float>(0, 0) * cos(fThetaLatitude) * cos(fThetaLongitude);
                p[j][1] = axe.at<float>(1, 0) * cos(fThetaLatitude) * sin(fThetaLongitude);
                p[j][2] = axe.at<float>(2, 0) * sin(fThetaLatitude);
                p[j][3] = 1.;
            }
        }

        // draw latitude
        for (int i = 0; i < pointGrid.rows; i++) {
            cv::Vec4f *p = pointGrid.ptr<cv::Vec4f>(i);
            // [0, 180]
            for (int j = 0; j < pointGrid.cols - 1; j++) {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = P * Twq * spherePt0;
                cv::Mat conicPt1 = P * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0),
                              conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0),
                              conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
            }
            // [180, 360]
            for (int j = 0; j < pointGrid.cols - 1; j++) {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = P * Twq * spherePt0;
                cv::Mat conicPt1 = P * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0),
                              conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0),
                              conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
            }
        }

        // draw longitude
        cv::Mat pointGrid_t = pointGrid.t();
        for (int i = 0; i < pointGrid_t.rows; i++) {
            cv::Vec4f *p = pointGrid_t.ptr<cv::Vec4f>(i);
            // [0, 180]
            for (int j = 0; j < pointGrid_t.cols - 1; j++) {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << p[j][0], p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << p[j + 1][0], p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = P * Twq * spherePt0;
                cv::Mat conicPt1 = P * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0),
                              conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0),
                              conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(im, pt0, pt1, sc, nLineWidth); // [0, 180]
            }
            // [180, 360]
            for (int j = 0; j < pointGrid_t.cols - 1; j++) {
                cv::Mat spherePt0 = (cv::Mat_<float>(4, 1) << -p[j][0], -p[j][1], p[j][2], p[j][3]);
                cv::Mat spherePt1 = (cv::Mat_<float>(4, 1) << -p[j + 1][0], -p[j + 1][1], p[j + 1][2], p[j + 1][3]);
                cv::Mat conicPt0 = P * Twq * spherePt0;
                cv::Mat conicPt1 = P * Twq * spherePt1;
                cv::Point pt0(conicPt0.at<float>(0, 0) / conicPt0.at<float>(2, 0),
                              conicPt0.at<float>(1, 0) / conicPt0.at<float>(2, 0));
                cv::Point pt1(conicPt1.at<float>(0, 0) / conicPt1.at<float>(2, 0),
                              conicPt1.at<float>(1, 0) / conicPt1.at<float>(2, 0));
                cv::line(im, pt0, pt1, sc, nLineWidth); // [180, 360]
            }
        }

        return im;
    }
} //namespace gtsam_soslam