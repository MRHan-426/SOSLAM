#ifndef SYSTEMSTATE_H
#define SYSTEMSTATE_H

#include "Constants.h"
#include "Map.h"

#include <vector>
#include <map>
#include <string>
#include <variant>
#include <iostream>
#include <optional>
#include <Eigen/Dense>
#include <limits>

#include <opencv4/opencv2/opencv.hpp> //4.7.0
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

namespace gtsam_soslam {
    typedef std::unordered_map<std::string, std::vector<ConstrainedDualQuadric>> GroundTruthSet;
    class Detection {
    public:
        std::string label;
        gtsam::Vector4 bounds;
        gtsam::Key pose_key;
        gtsam::Key quadric_key;

        bool operator==(const Detection &other) const {
            if (label == other.label && bounds == other.bounds && pose_key == other.pose_key &&
                quadric_key == other.quadric_key) {
                return true;
            } else {
                return false;
            }
        }

        Detection(const Detection &other) = default;

        Detection &operator=(const Detection &other) {
            if (this != &other) {
                label = other.label;
                bounds << other.bounds[0], other.bounds[1], other.bounds[2], other.bounds[3];
                pose_key = other.pose_key;
                quadric_key = other.quadric_key;
            }
            return *this;
        }

        explicit Detection(std::string label = "None", gtsam::Vector4 bounds = gtsam::Vector4::Zero(),
                           gtsam::Key pose_key = 0, gtsam::Key quadric_key = 66666) : label(std::move(label)),
                                                                                      bounds(std::move(bounds)),
                                                                                      pose_key(pose_key),
                                                                                      quadric_key(quadric_key) {}
    };

    class StepState
    {
    private:
        bool image_ready_=false;
    public:
        int i;
        gtsam::Key pose_key;
        cv::Mat rgb;
        cv::Mat depth;
        gtsam::Pose3 odom;
        std::vector<Detection> detections;
        std::vector<Detection> new_associated;
        std::vector<std::vector<std::pair<double, double>>> nearest_edge_point;

        explicit StepState(int i = 0) : i(i), pose_key(gtsam::Symbol('x', i)) {
        }

        StepState(const StepState &other) = default;

        StepState &operator=(const StepState &other) {
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
        void imageprepared(){ image_ready_=true;}
        bool needSave() const{return image_ready_;}
        void imageSaved(){ image_ready_=false;}
        bool isValid() const
        {
            return i != 0;
        }
    };

    class SoSlamState {
    public:
        Map* mpMap;
        gtsam::Pose3 initial_pose_;
        gtsam::Matrix noise_prior_;
        gtsam::Matrix noise_odom_;
        gtsam::Matrix noise_boxes_;
        bool optimizer_batch_;
        gtsam::ISAM2 isam_optimizer_;
        gtsam::LevenbergMarquardtParams optimizer_params_;
        gtsam::ISAM2Params isam_params_;
        std::vector<Detection> associated_;
        std::vector<Detection> unassociated_;
        std::map<gtsam::Key, std::string> labels_;
        gtsam::NonlinearFactorGraph graph_;
        gtsam::Values estimates_;
        // double calib_depth_;
        gtsam::Cal3_S2 calib_rgb_;
        StepState prev_step;
        StepState this_step;
        GroundTruthSet groundTruthes;
        string dataset;


        explicit SoSlamState(const string dataset, Map *mMap, const gtsam::Pose3 &initial_pose = Constants::POSES[0], \
        const bool &optimizer_batch = true, const GroundTruthSet gTruthes = Constants::groundTruthes())
                :   dataset(dataset),
                    mpMap(mMap),
                    initial_pose_(initial_pose),
                    optimizer_batch_(optimizer_batch),
                    groundTruthes(gTruthes){

            isam_params_.relinearizeThreshold = 0.05;
            isam_params_.relinearizeSkip = 10;
            isam_params_.enableRelinearization = true;
            isam_params_.evaluateNonlinearError = true;
            isam_params_.factorization = gtsam::ISAM2Params::QR;
            isam_optimizer_ = gtsam::ISAM2(isam_params_);

            optimizer_params_.setAbsoluteErrorTol(1e-8);
            optimizer_params_.setRelativeErrorTol(1e-8);
            optimizer_params_.lambdaInitial = 1e-5;
            optimizer_params_.lambdaFactor = 10.0;
            optimizer_params_.verbosity = gtsam::NonlinearOptimizerParams::ERROR;
//        optimizer_params_.verbosityLM = gtsam::LevenbergMarquardtParams::LAMBDA;
//        optimizer_params_.linearSolverType = gtsam::NonlinearOptimizerParams::Iterative;
//        auto subgraph_params = boost::shared_ptr<gtsam::IterativeOptimizationParameters>();
//        optimizer_params_.setIterativeParams(subgraph_params);
    }
};
} //gtsam_soslam
#endif // SYSTEMSTATE_H
