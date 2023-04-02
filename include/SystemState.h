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
#include <limits>

#include <gtsam/base/Vector.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/GaussNewtonOptimizer.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/geometry/Cal3.h>


namespace gtsam_soslam{

class Detection {
public:
    std::string label;
    gtsam::Vector4 bounds;
    gtsam::Key pose_key;
    gtsam::Key quadric_key;
    
    // TODO: Not sure if it will work eventually
    bool operator==(const Detection& other) const {

        if (label == other.label && bounds == other.bounds && pose_key == other.pose_key && quadric_key == other.quadric_key)
        {
//            std::cout<<"true"<<std::endl;
            return true;
        }
        else
        {
//            std::cout<<"false"<<std::endl;
            return false;
        }
    }

    Detection& operator=(const Detection& other) {
        if (this != &other) {
            label = other.label;
            bounds << other.bounds[0], other.bounds[1], other.bounds[2], other.bounds[3];
            pose_key = other.pose_key;
            quadric_key = other.quadric_key;
        }
        return *this;
    }

    explicit Detection(std::string label = "", gtsam::Vector4 bounds = gtsam::Vector4::Zero(), gtsam::Key pose_key = 0, gtsam::Key quadric_key = 66666) :
            label(std::move(label)), bounds(std::move(bounds)), pose_key(pose_key), quadric_key(quadric_key) {}
};

class StepState {
public:
    int i;
    gtsam::Key pose_key;
    gtsam::Vector3 rgb;
    gtsam::Matrix3 depth;
    gtsam::Pose3 odom;
    std::vector<Detection> detections;
    std::vector<Detection> new_associated;

    explicit StepState(int i = 0) : i(i), pose_key(gtsam::Symbol('x', i)) {
    }

    StepState& operator=(const StepState& other) {
        if (this != &other) {
            i = other.i;
            pose_key = other.pose_key;
            rgb = other.rgb;
            depth = other.depth;
            odom = other.odom;
            detections = other.detections;
            new_associated = other.new_associated;
        }
        return *this;
    }

    bool isValid() const {
        // TODO:Some Issues
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
//    gtsam::LevenbergMarquardtOptimizer optimizer_;
    gtsam::LevenbergMarquardtParams optimizer_params_;
    // std::variant<gtsam::ISAM2, gtsam::LevenbergMarquardtOptimizer> optimizer_;
    // std::variant<gtsam::ISAM2Params, gtsam::LevenbergMarquardtParams> optimizer_params_;
    std::vector<Detection> associated_;
    std::vector<Detection> unassociated_;
    std::map<gtsam::Key, std::string> labels_;
    gtsam::NonlinearFactorGraph graph_;
    gtsam::Values estimates_;
    // double calib_depth_;
    // boost::shared_ptr<gtsam::Cal3_S2> calib_rgb_;
    gtsam::Cal3_S2 calib_rgb_;
    StepState prev_step;
    StepState this_step;

    explicit SoSlamState(const gtsam::Pose3& initial_pose = Constants::POSES[0], const bool& optimizer_batch = true)
        : initial_pose_(initial_pose),
          optimizer_batch_(optimizer_batch)
    {
        optimizer_params_ = gtsam::LevenbergMarquardtParams();
        noise_prior_ = gtsam::Matrix(6,6);
        noise_odom_ = gtsam::Matrix(6,6);
        noise_boxes_ = gtsam::Matrix(4,4);
        noise_prior_ = 0.00 * noise_prior_.setIdentity();
        noise_odom_ = 0.01 * noise_prior_.setIdentity();
        noise_boxes_ = 3.00 * noise_boxes_.setIdentity();
    }
};
} //gtsam_soslam
#endif // SYSTEMSTATE_H
