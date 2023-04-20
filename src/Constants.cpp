#include "Constants.h"

namespace gtsam_soslam {
    namespace Constants {
        std::vector<gtsam::Pose3> initPoses() {
            std::vector<gtsam::Point3> points{
                    {10,  0,   0},
                    {0,   -10, 0},
                    {-10, 0,   0},
                    {0,   10,  0},
//            {10, 0, 0},
            };

            std::vector<gtsam::Pose3> poses;
            for (const auto &point: points) {
                auto camera = gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(point, gtsam::Point3(0, 0, 0),
                                                                           gtsam::Point3(0, 0, 1), gtsam::Cal3_S2());
                poses.push_back(camera.pose());
            }

            return poses;
        }
        std::unordered_map<std::string, std::vector<ConstrainedDualQuadric>> groundTruthes() { //sequence based on quadrics
            std::unordered_map<std::string, std::vector<ConstrainedDualQuadric>> groundtruthes;

            groundtruthes["Fr2_Desk"]=groundTruthQuadrics_scene1();
            groundtruthes["Fr1_Desk2"]=groundTruthQuadrics_scene3();
            groundtruthes["Fr2_Dishes"]=groundTruthQuadrics_scene2();
            return groundtruthes;
        }
        std::vector<ConstrainedDualQuadric> groundTruthQuadrics_scene1() { //sequence based on quadrics
            std::vector<ConstrainedDualQuadric> quadrics{
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 15 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {1.25, -1.1, 0.98}), {0.09, 0.28, 0.25}), //monitor
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.95, -1.15, 0.79}), {0.11, 0.22, 0.02}), //keyboard
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.77, -1.4, 0.78}), {0.06, 0.03, 0.03}), //mouse
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, -2 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {1.2, -1.78, 0.80}), {0.16, 0.12, 0.02}), //book
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.78, -0.65, 0.85}), {0.04, 0.04, 0.09}), //cup
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.90, -1.78, 0.79}), {0.075, 0.075, 0.05}), //tape
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {2.4, -1.2, 0.35}), {0.23, 0.23, 0.46}) //chair
//                    ConstrainedDualQuadric(gtsam::Pose3(
//                    gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 20 * M_PI / 180.0, 0 * M_PI / 180.0),
//                    {1.25, -1.07, 0.98}), {0.06, 0.21, 0.19}), //monitor
//                    ConstrainedDualQuadric(gtsam::Pose3(
//                    gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
//                    {1, -1.09, 0.77}), {0.105458, 0.22, 0.0102198}), //keyboard
//                    ConstrainedDualQuadric(gtsam::Pose3(
//                    gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
//                    {0.8, -1.36, 0.775}), {0.066934, 0.0370221, 0.0324774}), //mouse
//                    ConstrainedDualQuadric(gtsam::Pose3(
//                    gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, -2 * M_PI / 180.0, 0 * M_PI / 180.0),
//                    {1.30, -1.75, 0.77}), {0.15, 0.1, 0.02}), //book
//                    ConstrainedDualQuadric(gtsam::Pose3(
//                    gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
//                    {0.8, -0.55, 0.8}), {0.04, 0.04, 0.09}), //cup
//                    ConstrainedDualQuadric(gtsam::Pose3(
//                    gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
//                    {0.95, -1.78, 0.77}), {0.075, 0.075, 0.025}) //tape

            };
            return quadrics;
        }
        std::vector<ConstrainedDualQuadric> groundTruthQuadrics_scene2() { //sequence based on quadrics
            std::vector<ConstrainedDualQuadric> quadrics{
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 20 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-0.55, -0.45, 0.8}), {0.07, 0.07, 0.04}), //bowl_smaller
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-0.60, 0.18, 0.80}), {0.08, 0.08, 0.05}), //bowl_bigger
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-0.35, -0.12, 0.80}), {0.05, 0.05, 0.05}), //cup
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, -2 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-4.4, 0.25, 0.55}), {0.23, 0.23, 0.46}), //chair
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-5.6, 2.86, 0.95}), {0.17, 0.20, 0.15}), //laptop
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(0 * M_PI / 180.0, 15 * M_PI / 180.0, -20 * M_PI / 180.0),
                            {1.60, -1.10, 1.07}), {0.09, 0.28, 0.25}) //monitor

            };
            return quadrics;
        }
        std::vector<ConstrainedDualQuadric> groundTruthQuadrics_scene3() { //sequence based on quadrics
            std::vector<ConstrainedDualQuadric> quadrics{
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, -30 * M_PI / 180.0),
                            {-0.2, 0.17, 0.81}), {0.16, 0.12, 0.04}), //white_book
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.07, 0.12, 0.81}), {0.16, 0.12, 0.04}), //cube_book
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.85, 1.76, 0.81}), {0.16, 0.12, 0.04}), //blue_book
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, -2 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {0.40, 0.15, 0.81}), { 0.16, 0.12, 0.04}), //ass_book
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 45 * M_PI / 180.0),
                            {1.69, 1.68, 0.89}), {0.03, 0.03, 0.1}), //bottle
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-0.70, 0.67, 0.47}), {0.23, 0.23, 0.46}), //chair

                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 50 * M_PI / 180.0),
                            {0.56, 0.70, 0.86}), {0.05, 0.05, 0.07}), //cup
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 20 * M_PI / 180.0),
                            {0.62, 1.23, 0.80}), {0.06, 0.03, 0.03}), //white_mouse
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 60 * M_PI / 180.0),
                            {-0.06, 0.54, 0.80}), {0.06, 0.03, 0.03}), //black_mouse
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, -45 * M_PI / 180.0),
                            {0.86, 0.14, 0.80}), {0.06, 0.08, 0.02}), //game_remote
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 10 * M_PI / 180.0),
                            {0.70, 0.98, 0.80}), {0.11, 0.22, 0.02}), //white_keyboard
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, -60 * M_PI / 180.0),
                            {-0.35, 1.03, 0.9}), {0.17, 0.20, 0.15}), //laptop
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 20 * M_PI / 180.0, 95 * M_PI / 180.0),
                            {-0.94, 2.08, 1.03}), {0.09, 0.28, 0.25}), //monitor
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 * M_PI / 180.0, 0 * M_PI / 180.0, 0 * M_PI / 180.0),
                            {-0.8, 1.82, 0.8}), {0.18, 0.23, 0.02}), //laptop2

            };
            return quadrics;
        }
        std::vector<ConstrainedDualQuadric> initQuadrics() {
            std::vector<ConstrainedDualQuadric> quadrics{
                    ConstrainedDualQuadric(gtsam::Pose3(), {3, 0.5, 0.5}),
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(30.0 * M_PI / 180.0, 30.0 * M_PI / 180.0, 30.0 * M_PI / 180.0),
                            {2, 1, -1}), {0.5, 4, 0.5})
//                        ConstrainedDualQuadric(gtsam::Pose3(gtsam::Rot3(), {3, 1.5, 0.5}), {1, 4, 1})
            };

//        std::cout<<"standard"<<gtsam::Rot3()<<std::endl;
//        std::cout<<"z90"<<gtsam::Rot3::Rz(30.0 * M_PI / 180.0)<<std::endl;
            return quadrics;
        }

        const std::vector<gtsam::Pose3> POSES = initPoses();
        const std::vector<ConstrainedDualQuadric> QUADRICS = initQuadrics();
    } //namespace Constants
} //namespace gtsam_soslam
