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
        std::vector<ConstrainedDualQuadric> groundTruthQuadrics() { //sequence based on quadrics
            std::vector<ConstrainedDualQuadric> quadrics{
                    ConstrainedDualQuadric(gtsam::Pose3(
                        gtsam::Rot3::RzRyRx(2 *M_PI/180.0, 20 *M_PI/180.0, 0* M_PI / 180.0),
                        { 1.25, -1.07, 0.98}), {0.06, 0.21, 0.19}), //monitor
                    ConstrainedDualQuadric(gtsam::Pose3(
                        gtsam::Rot3::RzRyRx(2 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                        {1, -1.09, 0.77}), {0.105458, 0.22, 0.0102198}), //keyboard
                    ConstrainedDualQuadric(gtsam::Pose3(
                        gtsam::Rot3::RzRyRx(2 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                        {0.8, -1.36, 0.775}), {0.066934, 0.0370221, 0.0324774}), //mouse
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 *M_PI/180.0, -2 *M_PI/180.0, 0 * M_PI / 180.0),
                            {1.30, -1.75, 0.77}), {0.15, 0.1, 0.02}), //book
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                            {0.8, -0.55, 0.8}), {0.04, 0.04, 0.09}), //cup
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(2 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                            {0.95, -1.78, 0.77}), {0.075, 0.075, 0.025}) //tape

            };

//        std::cout<<"standard"<<gtsam::Rot3()<<std::endl;
//        std::cout<<"z90"<<gtsam::Rot3::Rz(30.0 * M_PI / 180.0)<<std::endl;
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
