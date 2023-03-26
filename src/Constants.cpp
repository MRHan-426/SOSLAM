#include "Constants.h"

namespace gtsam_soslam{
namespace Constants {
    std::vector<gtsam::Pose3> initPoses() {
        std::vector<gtsam::Point3> points{
            {10, 0, 0},
            {0, -10, 0},
            {-10, 0, 0},
            {0, 10, 0},
            {10, 0, 0},
        };

        std::vector<gtsam::Pose3> poses;
        for (const auto& point : points) {
            auto camera = gtsam::PinholeCamera<gtsam::Cal3_S2>::Lookat(point, gtsam::Point3(0, 0, 0), gtsam::Point3(0, 0, 1), gtsam::Cal3_S2());
            poses.push_back(camera.pose());
        }

        return poses;
    }

    std::vector<ConstrainedDualQuadric> initQuadrics() {
        std::vector<ConstrainedDualQuadric> quadrics{
            ConstrainedDualQuadric(gtsam::Pose3(), {1, 1, 1}),
            ConstrainedDualQuadric(gtsam::Pose3(gtsam::Rot3(), {1, 1, 1}), {1, 1, 1})
        };

        return quadrics;
    }
    const std::vector<gtsam::Pose3> POSES = initPoses();
    const std::vector<ConstrainedDualQuadric> QUADRICS = initQuadrics();
} //namespace Constants
} //namespace gtsam_soslam
