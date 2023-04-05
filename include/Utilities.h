/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Utilities.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a namespace providing a number of useful functions
 */

#pragma once
#include <map>
#include <set>
#include <vector>
#include <utility>

#include "ConstrainedDualQuadric.h"
#include "SoSlam.h"
#include "BoundingBoxFactor.h"
#include "SemanticScaleFactor.h"
#include "PlaneSupportingFactor.h"
#include "Evaluation.h"

#include <gtsam/base/Matrix.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <boost/optional.hpp>
#include <Eigen/Dense>

namespace gtsam_soslam
{
    namespace utils
    {

        /**
         * Returns the real roots of the polynomial
         * If disc > 0: 2 solutions
         * If disc == 0: 1 real solution
         * If disc < 0: 2 imaginary solutions
         */
        gtsam::Vector2 solvePolynomial(const double &a, const double &b,
                                       const double &c);

        gtsam::Vector2 getConicPointsAtX(
            const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x);

        gtsam::Vector2 getConicPointsAtY(
            const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y);

        /** Interpolate poses */
        gtsam::Pose3 interpolate(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2,
                                 const double &percent);

        /**
         * Converts Pose3 to Matrix and provides optional jacobians
         * https://atmos.washington.edu/~dennis/MatrixCalculus.pdf
         * https://en.wikipedia.org/wiki/Kronecker_product
         * https://www.math.uwaterloo.ca/~hwolkowi/matrixcookbook.pdf
         * https://people.maths.ox.ac.uk/gilesm/files/NA-08-01.pdf
         * Some Theorems on Matrix Differentiation with Special Reference to Kronecker
         * Matrix Products (H. Neudecker, 1969) A tutorial on SE(3) transformation
         * parameterizations and on-manifold optimization Jose-Luis Blanco (p.35)
         * -
         * https://jinyongjeong.github.io/Download/SE3/jlblanco2010geometry3d_techrep.pdf
         * http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
         */
        gtsam::Matrix44 matrix(const gtsam::Pose3 &pose,
                               gtsam::OptionalJacobian<16, 6> H = boost::none);

        /**
         * Performs the kronecker product
         * See: https://en.wikipedia.org/wiki/Kronecker_product
         */
        gtsam::Matrix kron(const gtsam::Matrix &m1, const gtsam::Matrix &m2);

        /**
         * Builds the orthogonal transpose vectorization matrix of an m by n matrix
         * See: http://www.ee.ic.ac.uk/hp/staff/dmb/matrix/special.html#VecTranspose
         */
        gtsam::Matrix TVEC(int m, int n);

        ConstrainedDualQuadric initialize_quadric_ray_intersection(
            const std::vector<gtsam::Pose3> &obs_poses,
            const std::vector<AlignedBox2> &boxes,
            SoSlamState &state);

        ConstrainedDualQuadric initialize_with_ssc_psc_bbs(
            const BoundingBoxFactor &bbs,
            const SemanticScaleFactor &ssc,
            const PlaneSupportingFactor &psc,
            const gtsam::Pose3 &camera_pose

        );

        // visualize
        void visualize(SoSlamState &state);

        std::pair<std::map<gtsam::Key, gtsam::Pose3>, std::map<gtsam::Key, ConstrainedDualQuadric>> ps_and_qs_from_values(const gtsam::Values &values);
        // newfactors
        gtsam::NonlinearFactorGraph new_factors(const gtsam::NonlinearFactorGraph &current,
                                                const gtsam::NonlinearFactorGraph &previous);
        // newvalues
        gtsam::Values new_values(const gtsam::Values &current, const gtsam::Values &previous);

        double area(const gtsam::Vector4&);

        double iou(const AlignedBox2&, const AlignedBox2&);
    } // namespace utils
} // namespace gtsam_soslam
