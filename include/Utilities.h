#pragma once

#include <map>
#include <set>
#include <vector>
#include <utility>
#include <iostream>

#include "ConstrainedDualQuadric.h"
#include "BoundingBoxFactor.h"
#include "SemanticScaleFactor.h"
#include "PlaneSupportingFactor.h"
#include "SymmetryFactor.h"
#include "Evaluation.h"
#include "Geometry.h"

#include <boost/optional.hpp>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <pcl/common/transforms.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

typedef pcl::PointXYZRGB PointT;
typedef pcl::PointCloud<PointT> PointCloudPCL;

namespace gtsam_soslam {
    namespace utils {

        gtsam::Vector2 solvePolynomial(const double &a, const double &b,
                                       const double &c);

        gtsam::Vector2 getConicPointsAtX(
                const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x);

        gtsam::Vector2 getConicPointsAtY(
                const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y);

        gtsam::Matrix44 matrix(const gtsam::Pose3 &pose,
                               gtsam::OptionalJacobian<16, 6> H = boost::none);

        gtsam::Matrix kron(const gtsam::Matrix &m1, const gtsam::Matrix &m2);

        gtsam::Matrix TVEC(int m, int n);

        ConstrainedDualQuadric initialize_quadric_ray_intersection(
                const std::vector<gtsam::Pose3> &obs_poses,
                const std::vector<AlignedBox2> &boxes,
                SoSlamState &state);

        ConstrainedDualQuadric initialize_with_ssc_psc_bbs_syc(
                const BoundingBoxFactor &bbs,
                const SemanticScaleFactor &ssc,
                const PlaneSupportingFactor &psc,
                const SymmetryFactor &syc,
                const gtsam::Pose3 &camera_pose);

        // visualize
        void visualize(SoSlamState &state);

        std::pair<std::map<gtsam::Key, gtsam::Pose3>, std::map<gtsam::Key, ConstrainedDualQuadric>>
        ps_and_qs_from_values(const gtsam::Values &values);

        // newfactors
        gtsam::NonlinearFactorGraph new_factors(const gtsam::NonlinearFactorGraph &current,
                                                const gtsam::NonlinearFactorGraph &previous);

        // newvalues
        gtsam::Values new_values(const gtsam::Values &current, const gtsam::Values &previous);

        double area(const gtsam::Vector4 &);

        double iou(const AlignedBox2 &, const AlignedBox2 &);

        Eigen::Matrix4d getTransformFromVector(Eigen::VectorXd& pose);

        PointCloud* pclToQuadricPointCloudPtr(PointCloudPCL::Ptr &pCloud);
    } // namespace utils
} // namespace gtsam_soslam
