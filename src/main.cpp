#include "main.h"


namespace gtsam_soslam {

    bool check_file_exists(const std::string& path) {
        std::ifstream infile(path);
        return infile.good();
    }

    void check_input_files_exist(const std::string& img_path, const std::string& dep_path,\
                                const std::string& xml_path, const std::string& calib_file,\
                                const std::string& odom_file){
        if (!check_file_exists(img_path))
            throw std::runtime_error("Image file not found");
        if (!check_file_exists(dep_path))
            throw std::runtime_error("Depth file not found");
        if (!check_file_exists(xml_path))
            throw std::runtime_error("XML file not found");
        if (!check_file_exists(calib_file))
            throw std::runtime_error("Calibration file not found");
        if (!check_file_exists(odom_file))
            throw std::runtime_error("Odometry file not found");
    }

    gtsam::Pose3 get_initial_pose(const std::string& path) {
        std::ifstream infile(path);
        if (!infile.good()) {
            throw std::runtime_error("Failed to read odom file");
        }
        std::string line;
        std::getline(infile, line);
        if (line.empty()) {
            throw std::runtime_error("Failed to read first line of odom file");
        }
        std::stringstream ss(line);
        std::vector<double> data;
        double d;
        while (ss >> d) {
            data.push_back(d);
        }
        if (data.size() != 8) {
            throw std::runtime_error("Invalid odometry data");
        }
        double tx = data[1];
        double ty = data[2];
        double tz = data[3];
        double qx = data[4];
        double qy = data[5];
        double qz = data[6];
        double qw = data[7];
        gtsam::Rot3 rot(gtsam::Quaternion(qw, qx, qy, qz));
        gtsam::Point3 t(tx, ty, tz);
        auto pose = gtsam::Pose3(rot, t);
        return pose;
    }

    void run(DataSource &data_source, BaseAssociator &associator, BaseDetector &detector, \
        const gtsam::Pose3 &initial_pose, const string example_type, const bool &use_3D_visualization, \
        const bool &optimizer_batch) {
        Map* mMap = new Map();
        string YamlFile = "../config.yaml";
        auto* state = new SoSlamState(example_type, mMap, initial_pose, optimizer_batch);
        auto *q = new SoSlam(
                data_source, //DataSource
                associator,//BaseAssociator
                detector, //BaseDetector
                mMap,
                state,
                YamlFile,
                initial_pose,//gtsam::Pose3 initial_pose
                optimizer_batch //bool optimizer_batch
        );
        SoSlamState *s = state;

        if (use_3D_visualization) {
            auto *mpFrameDrawer = new FrameDrawer(s);
            auto *mpMapDrawer = new MapDrawer(s, mMap, YamlFile);
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
    bool use_3D_visualization = false;
    string example_type = "Fr2_Desk";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--dummy") == 0)
            example_type = "dummy";
        else if (strcmp(argv[i], "--Fr2_Desk") == 0)
            example_type = "Fr2_Desk";
        else if (strcmp(argv[i], "--Fr1_Desk2") == 0)
            example_type = "Fr1_Desk2";
        else if (strcmp(argv[i], "--Fr2_Dishes") == 0)
            example_type = "Fr2_Dishes";
        else if (strcmp(argv[i], "--3d") == 0)
            use_3D_visualization = true;
    }

    if (example_type == "dummy") {
        const gtsam::Pose3 &initial_pose = gtsam_soslam::Constants::POSES[0];
        gtsam_soslam::DummyData data_source;
        gtsam_soslam::DummyAssociator associator;
        gtsam_soslam::DummyDetector detector;
        gtsam_soslam::run(data_source, associator, detector, initial_pose, example_type, use_3D_visualization, true);
    } else {
        const std::string &img_path = "../input/img/";
        const std::string &dep_path = "../input/depth/";
        const std::string &xml_path = "../input/xml/";
        const std::string &calib_file = "../input/calib_params.txt";
        const std::string &odom_file = "../input/odom.txt";
        gtsam_soslam::check_input_files_exist(img_path, dep_path, xml_path, calib_file, odom_file);
        std::cout << "All files exist" << std::endl;
        gtsam_soslam::HandMadeAssociator associator;
        gtsam_soslam::HandMadeDetector detector(xml_path);
        gtsam_soslam::HandMadeData data_source(img_path, dep_path, xml_path, calib_file, odom_file);
        const gtsam::Pose3 &initial_pose = gtsam_soslam::get_initial_pose(odom_file);
        gtsam_soslam::run(data_source, associator, detector, initial_pose, example_type, use_3D_visualization, false);
    }

    return 0;
}