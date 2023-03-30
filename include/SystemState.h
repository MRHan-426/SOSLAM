#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H
#include "Constants.h"

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <iostream>
#include <optional>
#include <Eigen/Dense>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>


namespace gtsam_soslam{

class Detection {
public:
    std::string label;
    gtsam::Vector4 bounds;
    int pose_key;
    int quadric_key;

    Detection(std::string label, gtsam::Vector4 bounds, int pose_key, int quadric_key=-1) :
        label(label), bounds(bounds), pose_key(pose_key), quadric_key(quadric_key) {}
};

class StepState {
public:
    int i;
    int pose_key;
    gtsam::Vector3 rgb;
    gtsam::Matrix3 depth;
    gtsam::Pose3 odom;
    std::vector<Detection> detections;
    std::vector<Detection> new_associated;

    StepState(int i = 0) : i(i), pose_key(gtsam::Symbol('x', i)) {}
    // TODO:Some Issues
    bool isValid() {
        return i != 0;
    }
};

class SoSlamState {
public:

    gtsam::Pose3 initial_pose_;
    gtsam::Matrix noise_prior_;
    gtsam::Matrix noise_odom_;
    gtsam::Matrix noise_boxes_;
    bool optimizer_batch_;
    gtsam::ISAM2 optimizer_;
    gtsam::ISAM2Params optimizer_params_;
    // std::variant<gtsam::ISAM2, gtsam::LevenbergMarquardtOptimizer> optimizer_;
    // std::variant<gtsam::ISAM2Params, gtsam::LevenbergMarquardtParams> optimizer_params_;
    std::vector<Detection> associated_;
    std::vector<Detection> unassociated_;
    std::map<int, std::string> labels_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values estimates_;
    double calib_depth_;
    gtsam::Matrix calib_rgb_;

    SoSlamState(const gtsam::Pose3& initial_pose = gtsam::Pose3(Constants::POSES[0].matrix()), const bool& optimizer_batch = true)
        : initial_pose_(initial_pose),
          optimizer_batch_(optimizer_batch)
    {
        noise_prior_ = gtsam::Matrix(6,6);
        noise_odom_ = gtsam::Matrix(6,6);
        noise_boxes_ = gtsam::Matrix(4,4);
        noise_prior_ = 0.00 * noise_prior_.setIdentity();
        noise_odom_ = 0.01 * noise_odom_.setIdentity();
        noise_boxes_ = 3.00 * noise_boxes_.setIdentity();
    }

};


// class SoSlamState {
// public:
//     SystemState system;
//     StepState prev_step;
//     StepState this_step;
//     SoSlamState(SystemState& system){};
// };
} //gtsam_soslam
#endif // SYSTEMSTATE_H
