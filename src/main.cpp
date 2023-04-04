#include "main.h"

namespace gtsam_soslam{
void run(DataSource& data_source,BaseAssociator& associator,BaseDetector& detector,const gtsam::Pose3& initial_pose) {

    SoSlam q(
            data_source, //DataSource
            associator,//BaseAssociator
            detector, //BaseDetector
            initial_pose,//gtsam::Pose3 initial_pose
            false //bool optimizer_batch
    );

    q.spin();
}
}

int main(int argc, char* argv[]) {
    // Check if the "--dummy" command-line argument is present
    bool use_dummy_data = false;
    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dummy") == 0) {
            use_dummy_data = true;
            break;
        }
    }
    if (use_dummy_data) {
        const gtsam::Pose3 &initial_pose = gtsam_soslam::Constants::POSES[0];
        gtsam_soslam::DummyData data_source;
        gtsam_soslam::DummyAssociator associator;
        gtsam_soslam::DummyDetector detector;
        gtsam_soslam::run(data_source, associator, detector, initial_pose);
    }
    else{
        gtsam::Pose3 pose(gtsam::Rot3::Quaternion(-0.4095, 0.6529, -0.5483, 0.3248), gtsam::Point3(-0.1546, -1.4445, 1.4773));
        const gtsam::Pose3 &initial_pose = pose;
        gtsam_soslam::HandMadeData data_source;
        gtsam_soslam::HandMadeAssociator associator;
        gtsam_soslam::HandMadeDetector detector;
        gtsam_soslam::run(data_source, associator, detector, initial_pose);
    }

    return 0;
}