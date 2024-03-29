/**
 * @file BoundingBoxFactor.h
 * @author Lachlan Nicholson, thanks for your great work
 * @modified by ROB530 group6
 * @Lastest modified on 19/04/2023
 */

#pragma once

#include "ConstrainedDualQuadric.h"
#include "AlignedBox2.h"

#include <boost/bind/bind.hpp>
#include <gtsam/base/numericalDerivative.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam_soslam {

    /**
     * @class BoundingBoxFactor
     * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
     * Projects the quadric at the current pose estimates,
     * Calculates the bounds of the dual conic,
     * and compares this to the measured bounding box.
     */
    class BoundingBoxFactor
            : public gtsam::NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric> {
    public:
        enum MeasurementModel {
            STANDARD,
            TRUNCATED
        }; ///< enum to declare which error function to use

    protected:
        AlignedBox2 measured_;                          ///< measured bounding box
        boost::shared_ptr<gtsam::Cal3_S2> calibration_; ///< camera calibration

        typedef NoiseModelFactor2 <gtsam::Pose3, ConstrainedDualQuadric>
                Base; ///< base class has keys and noisemodel as private members

        MeasurementModel measurementModel_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /// @name Constructors and named constructors
        /// @{

        /** Default constructor */
        BoundingBoxFactor()
                : measured_(0., 0., 0., 0.), measurementModel_(STANDARD) {};

        BoundingBoxFactor(const AlignedBox2 &measured,
                          const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                          const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                          const gtsam::SharedNoiseModel &model,
                          const MeasurementModel &errorType = STANDARD)
                : Base(model, poseKey, quadricKey),
                  measured_(measured),
                  calibration_(calibration),
                  measurementModel_(errorType) {};

        /** Constructor from measured box, calbration, dimensions and posekey,
         * quadrickey, noisemodel */
        BoundingBoxFactor(const AlignedBox2 &measured,
                          const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                          const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                          const gtsam::SharedNoiseModel &model,
                          const std::string &errorString)
                : Base(model, poseKey, quadricKey),
                  measured_(measured),
                  calibration_(calibration) {
            if (errorString == "STANDARD") {
                measurementModel_ = STANDARD;
            } else if (errorString == "TRUNCATED") {
                measurementModel_ = TRUNCATED;
            } else {
                throw std::logic_error(
                        "The error type \"" + errorString +
                        "\" is not a valid option for initializing a BoundingBoxFactor");
            }
        }

        /// @}
        /// @name Class accessors
        /// @{

        /** Returns the measured bounding box */
        AlignedBox2 measurement() const { return AlignedBox2(measured_.vector()); }

        gtsam::Key poseKey() const { return key1(); } // Returns the pose key

        gtsam::Key objectKey() const { return key2(); } // Returns the object/landmark key


        /**
         * Evaluate the error between a quadric and 3D pose
         * @param pose the 6DOF camera position
         * @param quadric the constrained dual quadric
         * @param H1 the derivative of the error wrt camera pose (4x6)
         * @param H2 the derivative of the error wrt quadric (4x9)
         */
        gtsam::Vector evaluateError(
                const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const;

        /** Evaluates the derivative of the error wrt pose */
        gtsam::Matrix evaluateH1(const gtsam::Pose3 &pose,
                                 const ConstrainedDualQuadric &quadric) const;

        /** Evaluates the derivative of the error wrt quadric */
        gtsam::Matrix evaluateH2(const gtsam::Pose3 &pose,
                                 const ConstrainedDualQuadric &quadric) const;

        /** Evaluates the derivative of the error wrt pose */
        gtsam::Matrix evaluateH1(const gtsam::Values &x) const;

        /** Evaluates the derivative of the error wrt quadric */
        gtsam::Matrix evaluateH2(const gtsam::Values &x) const;

        /// @}
        /// @name Testable group traits
        /// @{

        /** Prints the boundingbox factor with optional string */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                   gtsam::DefaultKeyFormatter) const override;

        /** Returns true if equal keys, measurement, noisemodel and calibration */
        bool equals(const BoundingBoxFactor &other, double tol = 1e-9) const;
    };

} // namespace gtsam_quadrics
