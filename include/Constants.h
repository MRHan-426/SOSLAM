
#pragma once

#include "ConstrainedDualQuadric.h"
#include <vector>
#include <unordered_map>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam_soslam {
    namespace Constants {
        // Function to initialize POSES
        std::vector<gtsam::Pose3> initPoses();
        std::unordered_map<std::string, std::vector<ConstrainedDualQuadric>> groundTruthes();
        std::vector<gtsam_soslam::ConstrainedDualQuadric> groundTruthQuadrics();

        std::vector<gtsam_soslam::ConstrainedDualQuadric> groundTruthQuadrics_scene1();
        std::vector<gtsam_soslam::ConstrainedDualQuadric> groundTruthQuadrics_scene2();
        std::vector<gtsam_soslam::ConstrainedDualQuadric> groundTruthQuadrics_scene3();
        // Function to initialize QUADRICS
        std::vector<gtsam_soslam::ConstrainedDualQuadric> initQuadrics();

        // Declare the constants
        extern const std::vector<gtsam::Pose3> POSES;
        extern const std::vector<gtsam_soslam::ConstrainedDualQuadric> QUADRICS;
    }
} //namespace gtsam_soslam
