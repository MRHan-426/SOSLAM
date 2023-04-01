#include "main.h"

namespace gtsam_soslam{
void run() {

    DummyData data_source;
    DummyAssociator associator;
    DummyDetector detector;
    gtsam::Pose3 initial_pose = Constants::POSES[0];

    SoSlam q(
            data_source, //DummyData data_source,
            associator,//DummyAssociator associator
            detector, //DummyDetector detector
            initial_pose,//gtsam::Pose3 initial_pose
            true //bool optimizer_batch
    );
    q.spin();
}
}

int main() {
    gtsam_soslam::run();
    return 0;
}