#include "main.h"


namespace gtsam_soslam {

    void run(DataSource &data_source, BaseAssociator &associator, BaseDetector &detector, \
        const gtsam::Pose3 &initial_pose, const bool &use_3D_visualization) {

        SoSlam *q = new SoSlam(
                data_source, //DataSource
                associator,//BaseAssociator
                detector, //BaseDetector
                initial_pose,//gtsam::Pose3 initial_pose
                false //bool optimizer_batch
        );
        SoSlamState *s = &(q->state_);

        if (use_3D_visualization) {
            string YamlFile = "../TUM2.yaml";
            FrameDrawer *mpFrameDrawer = new FrameDrawer(s);
            auto *mpMapDrawer = new MapDrawer(s, YamlFile);
            auto *mpViewer = new Viewer(s, mpFrameDrawer, mpMapDrawer, YamlFile);
            auto *mptViewer = new thread(&Viewer::Run, mpViewer);
            auto *mptLocalMapping = new thread(&SoSlam::spin, q);
            mptViewer->join();
            mptLocalMapping->join();
        } else {
            q->spin();
        }
    }
}


int main(int argc, char *argv[]) {
    // Check if the "--dummy" command-line argument is present
    bool use_dummy_data = false;
    bool use_3D_visualization = false;

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dummy") == 0) {
            use_dummy_data = true;
        } else if (strcmp(argv[i], "--3d") == 0) {
            use_3D_visualization = true;
        }
    }

    if (use_dummy_data) {
        const gtsam::Pose3 &initial_pose = gtsam_soslam::Constants::POSES[0];
        gtsam_soslam::DummyData data_source;
        gtsam_soslam::DummyAssociator associator;
        gtsam_soslam::DummyDetector detector;
        gtsam_soslam::run(data_source, associator, detector, initial_pose, use_3D_visualization);
    } else {
        const std::string &img_path = "../input/img/";
        const std::string &xml_path = "../input/xml/";
        const std::string &calib_file = "../input/calib_params.txt";
        const std::string &odom_file = "../input/odom.txt";
        gtsam::Pose3 pose(gtsam::Rot3::Quaternion(-0.4095, 0.6529, -0.5483, 0.3248),
                          gtsam::Point3(-0.1546, -1.4445, 1.4773));
        const gtsam::Pose3 &initial_pose = pose;

        gtsam_soslam::HandMadeData data_source(img_path, xml_path, calib_file, odom_file);
        gtsam_soslam::HandMadeAssociator associator;
        gtsam_soslam::HandMadeDetector detector(xml_path);
        gtsam_soslam::run(data_source, associator, detector, initial_pose, use_3D_visualization);
    }

    return 0;
}