//
// Created by ziqihan on 4/8/23.
//

#ifndef GTSAM_SOSLAM_SYMMETRYFACTOR_H
#define GTSAM_SOSLAM_SYMMETRYFACTOR_H
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
#include <opencv2/opencv.hpp>

namespace gtsam_soslam
{
    /**
     * SymmetryFactor
     * factor between Pose3 and ConstrainedDualQuadric
     */
    class SymmetryFactor
        : public gtsam::NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric>
    {
    public:
        enum MeasurementModel
        {
            STANDARD,
            TRUNCATED
        };

    protected:
        cv::Mat image_;
        AlignedBox2 measured_;
        std::string label_;
        boost::shared_ptr<gtsam::Cal3_S2> calibration_; //< camera calibration
        typedef NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric> Base;
        MeasurementModel measurementModel_;
        std::vector<std::vector<std::pair<int, int>>> nearest_edge_point_;
        int sigma_scc_;

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW

        /** Default constructor */
        SymmetryFactor()
            : label_("None"), measurementModel_(STANDARD), sigma_scc_(1){};

        SymmetryFactor(const AlignedBox2 &measured,
                       const cv::Mat &image,
                       const std::string &label,
                       const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                       const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                       const gtsam::SharedNoiseModel &model,
                       const std::vector<std::vector<std::pair<int, int>>> &nearest_edge_point,
                       const MeasurementModel &errorType = STANDARD,

                       const int &sigma_scc = 1)
            : Base(model, poseKey, quadricKey),
              image_(image),
              measured_(measured),
              label_(label),
              calibration_(calibration),
              measurementModel_(errorType),
              nearest_edge_point_(nearest_edge_point),
              sigma_scc_(sigma_scc){};

        SymmetryFactor(const AlignedBox2 &measured,
                       const cv::Mat &image,
                       const std::string &label,
                       const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                       const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                       const gtsam::SharedNoiseModel &model,
                       const std::string &errorString,
                       const std::vector<std::vector<std::pair<int, int>>> &nearest_edge_point,
                       const int &sigma_scc = 1)
            : Base(model, poseKey, quadricKey),
              image_(image),
              measured_(measured),
              label_(label),
              calibration_(calibration),
              nearest_edge_point_(nearest_edge_point),
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
                    "\" is not a valid option for initializing a SymmetryFactor");
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

        /** Prints the SymmetryFactor with optional string */
        void print(const std::string &s = "",
                   const gtsam::KeyFormatter &keyFormatter =
                       gtsam::DefaultKeyFormatter) const override;

        /** Returns true if equal keys, measurement, noisemodel and calibration */
        bool equals(const SymmetryFactor &other, double tol = 1e-9) const;
    };

} // namespace gtsam_soslam

// Add to testable group
template <>
struct gtsam::traits<gtsam_soslam::SymmetryFactor>
    : public gtsam::Testable<gtsam_soslam::SymmetryFactor>
{
};
#endif // GTSAM_SOSLAM_SYMMETRYFACTOR_H
