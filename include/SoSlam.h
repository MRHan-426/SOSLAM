#pragma once

#include "SystemState.h"
#include "Utilities.h"
#include "ConstrainedDualQuadric.h"
#include "AlignedBox2.h"
#include "BoundingBoxFactor.h"
#include "DummyExample.h"

#include <optional>
#include <functional>
#include <stdexcept>
#include <variant>
#include <limits>

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>



namespace gtsam_soslam{
class SoSlam {
public:
    // TODO：Class Associator, DataSource, DummyDetector
    DummyData data_source_;
    DummyAssociator associator_;
    DummyDetector detector_;
    // VisualOdometry visual_odometry_;
    gtsam::Pose3 initial_pose_;
    bool optimizer_batch_;
    SoSlamState state_; 


    SoSlam(
        DummyData data_source,
        DummyAssociator associator,
        DummyDetector detector,
        //VisualOdometry visual_odometry = std::nullptr, //Optional
        const gtsam::Pose3& initial_pose = gtsam::Pose3(Constants::POSES[0].matrix()),
        const bool& optimizer_batch = true
    );

// private:
    void guess_initial_values();
    void spin();
    void step();
    void reset();

};
}//namespace gtsam_soslam