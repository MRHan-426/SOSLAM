#include "main.h"

namespace gtsam_soslam{
void run() {

    DummyData data_source;
    DummyDetector detector;
    DummyAssociator associator;
    gtsam::Pose3 initial_pose = Constants::POSES[0];

    SoSlam q(
        data_source, //DummyData data_source,
        detector, //DummyDetector detector
        associator,//DummyAssociator associator
        initial_pose,//gtsam::Pose3 initial_pose
        true //bool optimizer_batch
    );
    q.spin();
}
} // namespace gtsam_soslam

int main() {
    gtsam_soslam::run();
    return 0;
}