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

namespace gtsam_soslam{
    namespace evaluate {

        // Function to calculate 2D IOU
        std::vector<double> iou_evaluation(SoSlamState &state);

        // Function to calculate Rot
        std::vector<gtsam::Vector3> rot_evaluation(SoSlamState &state);

    }
} //namespace gtsam_soslam
#endif //GTSAM_SOSLAM_EVALUATION_H
