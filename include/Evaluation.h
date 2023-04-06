//
// Created by ziqihan on 4/5/23.
//

#ifndef GTSAM_SOSLAM_EVALUATION_H
#define GTSAM_SOSLAM_EVALUATION_H

#include "ConstrainedDualQuadric.h"
#include "QuadricProjectionException.h"
#include "DualConic.h"
#include "QuadricCamera.h"
#include "SystemState.h"
#include "Utilities.h"

#include <vector>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/geometry/PinholeCamera.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <opencv4/opencv2/opencv.hpp> // 4.7.0

namespace gtsam_soslam{
    namespace evaluate {

        // visualize
        void visualize(SoSlamState &state);

        // Function to calculate 2D IOU
        std::vector<double> iou_evaluation(const SoSlamState &state);

        // Function to calculate Rot
        std::vector<gtsam::Vector3> rot_evaluation(SoSlamState &state);
        void draw_ellipsoid(const SoSlamState &state, const DualConic &dualConic);

    }
} //namespace gtsam_soslam
#endif //GTSAM_SOSLAM_EVALUATION_H
