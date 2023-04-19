/**
 * @file DualConic.h
 * @author Lachlan Nicholson, thanks for your great work
 * @modified by ROB530 group6
 * @Lastest modified on 19/04/2023
 */

#pragma once
#include "AlignedBox2.h"
#include <cmath>
#include <iostream>
#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/base/numericalDerivative.h>

namespace gtsam_soslam {

/**
 * @class DualConic
 * A dual conic (Matrix33)
 */
    class DualConic {
    protected:
        gtsam::Matrix33 dC_;  ///< 3x3 matrix of the quadratic equation

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** Default constructor: unit circle at origin */
        DualConic();

        /** Constructor from 3x3 matrix */
        DualConic(const gtsam::Matrix33 &dC) : dC_(dC) {};

        /** Create ellipse from 2D pose and axis lengths */
        DualConic(const gtsam::Pose2 &pose, const gtsam::Vector2 &radii);

        /// @}
        /// @name Class methods
        /// @{

        /** Return 3x3 conic matrix */
        gtsam::Matrix33 matrix(void) const { return dC_; }

        /** Return normalized dual conic */
        DualConic normalize(void) const;

        /**
         * Returns the standard 2D bounds on the image plane
         * with no consideration for image dimensions
         */
        AlignedBox2 bounds(gtsam::OptionalJacobian<4, 9> H = boost::none) const;

        /**
         * Returns the bounds as an object detector would see
         * Carefully handling the intersection with the image boundaries
         * NOTE: assumes conic is elliptical and non-degenerate (will throw
         * std::runtimeerror) FAILS: if quadric is not visible
         */
        AlignedBox2 smartBounds(const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                                gtsam::OptionalJacobian<4, 9> H = boost::none) const;

        /**
         * Returns true if conic section is degenerate
         * Using det(C) as opposed to sign(eigenvalues)
         */
        bool isDegenerate(void) const;

        /**
         * Returns true if conic section is elliptical or circular
         * Internally calculates degeneracy
         */
        bool isEllipse(void) const;

        /**
         * Returns true if dual conic contains the point
         * Points on the edge of the conic are considered contained
         */
        bool contains(const gtsam::Point2 &p) const;

        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the dual conic with optional string */
        void print(const std::string &s = "") const;

        /** Compares two dual conics accounting for normalization */
        bool equals(const DualConic &other, double tol = 1e-9) const;

        /// @}
    };

}  // namespace gtsam_soslam

/** \cond PRIVATE */
// Add DualConic to Testable group
template<>
struct gtsam::traits<gtsam_soslam::DualConic>
        : public gtsam::Testable<gtsam_soslam::DualConic> {
};
/** \endcond */
