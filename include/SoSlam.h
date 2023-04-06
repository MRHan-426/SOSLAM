#pragma once

#include "SystemState.h"
#include "DummyExample.h"
#include "Evaluation.h"
#include "Utilities.h"
#include "AlignedBox2.h"
#include "ConstrainedDualQuadric.h"
#include "BoundingBoxFactor.h"
#include "SemanticScaleFactor.h"
#include "PlaneSupportingFactor.h"

#include <optional>
#include <functional>
#include <stdexcept>
#include <variant>
#include <limits>

#include <opencv4/opencv2/opencv.hpp> //4.7.0
#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam_soslam
{
    class SoSlam
    {
    public:
        // TODO：Class Associator, DataSource, DummyDetector
        DataSource& data_source_;
        BaseAssociator& associator_;
        BaseDetector& detector_;
        // VisualOdometry visual_odometry_;
        gtsam::Pose3 initial_pose_;
        bool optimizer_batch_;
        SoSlamState state_;

        SoSlam(
            DataSource& data_source,
            BaseAssociator& associator,
            BaseDetector& detector,
            // VisualOdometry visual_odometry = std::nullptr, //Optional
            const gtsam::Pose3 &initial_pose = gtsam::Pose3(Constants::POSES[0].matrix()),
            const bool &optimizer_batch = true);

        // private:
        void guess_initial_values();
        void spin();
        void step();
        void reset();
        std::tuple<BoundingBoxFactor, SemanticScaleFactor, PlaneSupportingFactor> add_detection_factors(const Detection &d, const gtsam::noiseModel::Robust::shared_ptr &noise_boxes,
                                                                                                        const gtsam::noiseModel::Robust::shared_ptr &noise_scc,
                                                                                                        const gtsam::noiseModel::Robust::shared_ptr &noise_psc);

        gtsam::Point3 project_2d_point_to_3d(const Detection &d, const gtsam::Pose3 &camera_pose);
        Eigen::MatrixXd pseudoInverse(const Eigen::MatrixXd& A);

    };
} // namespace gtsam_soslam