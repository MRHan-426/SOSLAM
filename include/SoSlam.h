#pragma once

#include "SystemState.h"
#include "DummyExample.h"
#include "Utilities.h"
#include "AlignedBox2.h"
#include "ConstrainedDualQuadric.h"
#include "BoundingBoxFactor.h"
#include "SemanticScaleFactor.h"
#include "PlaneSupportingFactor.h"
#include "SymmetryFactor.h"
#include "../src/dense_builder/builder.h"
#include "Map.h"

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
// #include "FrameDrawer.h"

//
// Created by ziqihan on 3/2/23.
//

namespace gtsam_soslam {
    class SoSlam {
    public:
        // TODO：Class Associator, DataSource, DummyDetector
        DataSource &data_source_;
        BaseAssociator &associator_;
        BaseDetector &detector_;
        // VisualOdometry visual_odometry_;
        //Map
        Map* mpMap;
        SoSlamState state_;
        Builder* mpBuilder;     // a dense pointcloud builder from visualization

        gtsam::Pose3 initial_pose_;
        bool optimizer_batch_;

        bool output_quadrics_image_;

        SoSlam(
                DataSource &data_source,
                BaseAssociator &associator,
                BaseDetector &detector,
                Map* mMap,
                // VisualOdometry visual_odometry = std::nullptr, //Optional
                SoSlamState state,
                const string &strSettingPath,
                const gtsam::Pose3 &initial_pose = gtsam::Pose3(Constants::POSES[0].matrix()),
                const bool &optimizer_batch = true,
                const bool &output_quadrics_image = false);

        // private:
        void guess_initial_values();

        void spin();

        void step();

        void reset();

        std::vector<std::vector<std::pair<double, double>>>
        findNearestEdge(std::vector<std::pair<double, double>> &feature_points, double max_x, double max_y);

        std::tuple<BoundingBoxFactor, SemanticScaleFactor, PlaneSupportingFactor, SymmetryFactor>
        add_detection_factors(const Detection &d, const gtsam::noiseModel::Robust::shared_ptr &noise_boxes,
                              const gtsam::noiseModel::Robust::shared_ptr &noise_scc,
                              const gtsam::noiseModel::Robust::shared_ptr &noise_psc,
                              const gtsam::noiseModel::Robust::shared_ptr &noise_syc);

        void updateInitialEstimates(const gtsam::NonlinearFactorGraph &graph, gtsam::Values &initialEstimates);

        void limitFactorGraphSize(gtsam::NonlinearFactorGraph &graph, size_t maxFactors);
    };
} // namespace gtsam_soslam