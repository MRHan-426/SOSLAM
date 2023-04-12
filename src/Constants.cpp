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
        std::vector<ConstrainedDualQuadric> groundTruthQuadrics() {
            std::vector<ConstrainedDualQuadric> quadrics{
                    ConstrainedDualQuadric(gtsam::Pose3(
                        gtsam::Rot3::RzRyRx(4.10157 *M_PI/180.0, 0 *M_PI/180.0, 0* M_PI / 180.0),
                        { 0.945999, -1.19049, 1.07373}), {0.0894989, 0.181131, 0.201142}),
                    ConstrainedDualQuadric(gtsam::Pose3(
                        gtsam::Rot3::RzRyRx(-6.47381 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                        {0.717539, -1.196, 0.943438}), {0.105458, 0.201747, 0.0102198}),
                    ConstrainedDualQuadric(gtsam::Pose3(
                        gtsam::Rot3::RzRyRx(-2.79001 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                        {0.769214, -1.39832, 0.788364}), {0.066934, 0.0370221, 0.0324774}),
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(-6.0229 *M_PI/180.0, 0.0565107 *M_PI/180.0, 0 * M_PI / 180.0),
                            {0.864449, -1.69637, 0.985251}), {0.139547, 0.0941773, 0.0180347}),
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(7.11472 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                            {0.889008, -0.512086, 0.7607}), {0.0839582, 0.0427019, 0.0943408}),
                    ConstrainedDualQuadric(gtsam::Pose3(
                            gtsam::Rot3::RzRyRx(-99.6177 *M_PI/180.0, 0 *M_PI/180.0, 0 * M_PI / 180.0),
                            {0.733733, -1.73657, 0.892947}), {0.0642692, 0.0227489, 0.0650887})

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
