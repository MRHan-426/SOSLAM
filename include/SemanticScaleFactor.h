//
// Created by ziqihan on 3/2/23.
//
#pragma once
#include "AlignedBox2.h"
#include "ConstrainedDualQuadric.h"

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

namespace gtsam_soslam
{
    /**
     * SemanticScaleFactor
     * factor between Pose3 and ConstrainedDualQuadric
     */
    class SemanticScaleFactor
            : public gtsam::NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric>
    {
    public:
        enum MeasurementModel
        {
            STANDARD,
            TRUNCATED
        };

    protected:
        std::string label_;
        boost::shared_ptr<gtsam::Cal3_S2> calibration_; //< camera calibration
        typedef NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric> Base;
        MeasurementModel measurementModel_;
        int sigma_scc_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** Default constructor */
        SemanticScaleFactor()
                : label_("None"),measurementModel_(STANDARD), sigma_scc_(1){};

        SemanticScaleFactor(const std::string &label,
                            const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                            const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                            const gtsam::SharedNoiseModel &model,
                            const MeasurementModel &errorType = STANDARD,
                            const int &sigma_scc = 1
                            )
                : Base(model, poseKey, quadricKey),
                  label_(label),
                  calibration_(calibration),
                  measurementModel_(errorType),
                  sigma_scc_(sigma_scc)
                  {};

        SemanticScaleFactor(const std::string &label,
                            const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                            const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                            const gtsam::SharedNoiseModel &model,
                            const std::string &errorString,
                            const int &sigma_scc = 1)
                : Base(model, poseKey, quadricKey),
                  label_(label),
                  calibration_(calibration),
                  sigma_scc_(sigma_scc)
        {
            if (errorString == "STANDARD")
            {
                measurementModel_ = STANDARD;
            }
            else if (errorString == "TRUNCATED")
            {
                measurementModel_ = TRUNCATED;
            }
            else
            {
                throw std::logic_error(
                        "The error type \"" + errorString +
                        "\" is not a valid option for initializing a SemanticScaleFactor");
            }
        }

        gtsam::Key poseKey() const { return key1(); } // Returns the pose key

        gtsam::Key objectKey() const { return key2(); } // Returns the object/landmark key

        /**
         * Evaluate the error between a quadric and 3D pose
         * @param pose the 6DOF camera position
         * @param quadric the constrained dualquadric
         * @param H1 the derivative of the error wrt camera pose (1x6)
         * @param H2 the derivative of the error wrt quadric (1x9)
         */
        gtsam::Vector evaluateError(
                const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric,
                boost::optional<gtsam::Matrix &> H1 = boost::none,
                boost::optional<gtsam::Matrix &> H2 = boost::none) const override;

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

        /** Prints the SemanticScaleFactor with optional string */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                   gtsam::DefaultKeyFormatter) const override;

        /** Returns true if equal keys, measurement, noisemodel and calibration */
        bool equals(const SemanticScaleFactor &other, double tol = 1e-9) const;
    };

} // namespace gtsam_soslam

// Add to testable group
template <>
struct gtsam::traits<gtsam_soslam::SemanticScaleFactor>
        : public gtsam::Testable<gtsam_soslam::SemanticScaleFactor>
{
};


