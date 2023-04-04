#include "main.h"


namespace gtsam_soslam{
std::thread* mptViewer;
std::thread* mptLocalMapping;
void run() {

    DummyData data_source;
    DummyAssociator associator;
    DummyDetector detector;
    const gtsam::Pose3& initial_pose = Constants::POSES[0];

    SoSlam * q= new SoSlam(
            data_source, //DummyData data_source,
            associator,//DummyAssociator associator
            detector, //DummyDetector detector
            initial_pose,//gtsam::Pose3 initial_pose
            true //bool optimizer_batch
    );
    SoSlamState* s = &(q->state_);
//    mpFrameDrawer = new FrameDrawer(s);
    string YamlFile = "../TUM2.yaml";
    MapDrawer* mpMapDrawer = new MapDrawer(s, YamlFile);
    FrameDrawer* mpFrameDrawer= nullptr;
    Viewer* mpViewer = new Viewer(s, mpFrameDrawer,mpMapDrawer,YamlFile);
//     if(bUseViewer)
    mptViewer = new thread(&Viewer::Run, mpViewer);
    mptLocalMapping = new thread(&SoSlam::spin,q);
    mptViewer->join();
    mptLocalMapping->join();
    // q.spin();
}
}

int main() {
    gtsam_soslam::run();
    return 0;
}