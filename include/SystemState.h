#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H

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
    gtsam::noiseModel::Diagonal::shared_ptr noise_prior_;
    gtsam::noiseModel::Diagonal::shared_ptr noise_odom_;
    gtsam::noiseModel::Diagonal::shared_ptr noise_boxes_;
    bool optimizer_batch_;
    std::variant<gtsam::ISAM2, gtsam::LevenbergMarquardtOptimizer> optimizer_;
    std::variant<gtsam::ISAM2Params, gtsam::LevenbergMarquardtParams> optimizer_params_;
    // std::optional<gtsam::NonlinearOptimizer> optimizer_;
    // std::optional<gtsam::NonlinearOptimizerParams> optimizer_params_;
    std::vector<Detection> associated_;
    std::vector<Detection> unassociated_;
    std::map<int, std::string> labels_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values estimates_;
    double calib_depth_;
    boost::optional<gtsam::Matrix> calib_rgb_;
    StepState prev_step;
    StepState this_step;
    SystemState(const gtsam::Pose3& initial_pose = gtsam::Pose3(Constants::POSES[0].matrix()), const bool& optimizer_batch = true)
        : initial_pose_(initial_pose),
          noise_prior_(gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0).finished())),
          noise_odom_(gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(6) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01).finished())),
          noise_boxes_(gtsam::noiseModel::Diagonal::Variances((gtsam::Vector(4) << 3.0, 3.0, 3.0, 3.0).finished())),
          optimizer_batch_(optimizer_batch)
    {
        if (optimizer_batch)
        {
            optimizer_params_ = gtsam::LevenbergMarquardtParams();
        }
        else
        {
            optimizer_params_ = gtsam::ISAM2Params();
        }
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
