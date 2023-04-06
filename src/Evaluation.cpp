//
// Created by ziqihan on 4/5/23.
//

// IOU
// The evaluation for shape used the Jaccard distance (1 - Intersection over Union)
// between the 3D axis-aligned bounding box of the estimated ellipsoid and the real
// bounding box after they are translated to the origin point.
// https://arxiv.org/pdf/2004.05303.pdf

#include "Evaluation.h"
using namespace cv;
namespace gtsam_soslam{
    namespace evaluate {

    // Function to calculate 2D IOU
    std::vector<double> iou_evaluation(const SoSlamState &state){
        auto pose_values = utils::ps_and_qs_from_values(state.estimates_).first;
        auto quadric_values = utils::ps_and_qs_from_values(state.estimates_).second;
        // use max posekey (latest pose)
        gtsam::Key max_key = 0;
        gtsam::Pose3 camera_pose;
        for (const auto& kv : pose_values) {
            if (kv.first > max_key) {
                max_key = kv.first;
                camera_pose = kv.second;
            }
        }
        boost::shared_ptr<gtsam::Cal3_S2> calibration(new gtsam::Cal3_S2(state.calib_rgb_));
        DualConic dualConic;
        AlignedBox2 result_bounds;
        std::vector<double> ious;

        for (const auto& quadric_value : quadric_values) {
            gtsam::Key quadric_key = quadric_value.first;
            ConstrainedDualQuadric quadric = quadric_value.second;
            auto detection = std::find_if(state.this_step.detections.begin(), state.this_step.detections.end(), [quadric_key](const Detection& d) {
                return d.quadric_key == quadric_key;
            });
//            std::cout << quadric.pose() << std::endl << quadric.radii()<< std::endl;
//            std::cout << camera_pose<< std::endl;

            dualConic = QuadricCamera::project(quadric, camera_pose, calibration);
            // check dual conic is valid for error function
            if (!dualConic.isEllipse()) {
                throw QuadricProjectionException("Projected Conic is non-ellipse");
            }
            result_bounds = dualConic.smartBounds(calibration);
//            result_bounds = dualConic.bounds();

            ious.push_back(utils::iou(result_bounds, AlignedBox2(detection->bounds)));
            // draw_ellipsoid(state, dualConic);
        }
        return ious;
    }

        void visualize(SoSlamState &state)
        {
            gtsam::Values values = state.estimates_;
            auto labels = state.labels_;
            std::vector<double> ious = iou_evaluation(state);
            int i = 1;
            for (auto iou : ious){
                std::cout << "quadric " << i << "IOU = " << iou << std::endl;
                i++;
            }
        }

    // Function to initialize QUADRICS
    void draw_ellipsoid(const SoSlamState &state, const DualConic &dualConic){
        //TODO
        auto parameters = dualConic.matrix();
        parameters = parameters / parameters(2, 2);
        auto A = parameters(0,0);
        auto B = parameters(0,1);
        auto C = parameters(1,1);
        auto D = parameters(0,1);
        auto E = parameters(2,1);
        auto F = parameters(2,2);

        Mat Cmat = (Mat_<double>(3, 3) << A, B, D, B, C, E, D, E, F);

        auto img = state.this_step.rgb;

        Mat eigenvalues, eigenvectors;
        eigen(Cmat, eigenvalues, eigenvectors);
        double lambda1 = eigenvalues.at<double>(0);
        double lambda2 = eigenvalues.at<double>(1);
        double a = sqrt(-1.0/lambda1);
        double b = sqrt(-1.0/lambda2);
        double cx = eigenvectors.at<double>(0, 2);
        double cy = eigenvectors.at<double>(1, 2);
        double angle = atan2(eigenvectors.at<double>(1, 0), eigenvectors.at<double>(0, 0)) * 180.0 / CV_PI;

        ellipse(img, Point(cx, cy), Size(a, b), angle, 0, 360, Scalar(0, 255, 0), 2);

        cv::imwrite("output/1.png", img);
    }
} //namespace evaluate
} //namespace gtsam_soslam
