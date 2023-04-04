#include "main.h"

namespace gtsam_soslam{
void run(DataSource& data_source,BaseAssociator& associator,BaseDetector& detector) {

//    DummyData data_source;
//    DummyAssociator associator;
//    DummyDetector detector;
    const gtsam::Pose3& initial_pose = Constants::POSES[0];

    SoSlam q(
            data_source, //DummyData data_source,
            associator,//DummyAssociator associator
            detector, //DummyDetector detector
            initial_pose,//gtsam::Pose3 initial_pose
            false //bool optimizer_batch
    );

    q.spin();
}
}

int main(int argc, char* argv[]) {


    gtsam_soslam::DummyData data_source;
    gtsam_soslam::DummyAssociator associator;
    gtsam_soslam::DummyDetector detector;
    gtsam_soslam::run(data_source, associator, detector);


    return 0;
}