//
// Created by ziqihan on 3/2/23.
//
#include "QuadricProjectionException.h"
#include "SemanticScaleFactor.h"
#include "SemanticTable.h"

#include <gtsam/base/numericalDerivative.h>
#include <boost/bind/bind.hpp>
#include <cmath>
#include <Eigen/Dense>
#include <iostream>

#define NUMERICAL_DERIVATIVE true

using namespace std;

namespace gtsam_soslam
{
    gtsam::Vector SemanticScaleFactor::evaluateError(
            const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric,
            boost::optional<gtsam::Matrix &> H1,
            boost::optional<gtsam::Matrix &> H2) const
    {
        // In the function, the camera pose and quadratic surface inputs are first checked for validity.
        // An exception is thrown if the quadratic is behind the camera or the camera is inside the object.
        try
        {
            // check pose-quadric pair
            if (quadric.isBehind(pose))
            {
                throw QuadricProjectionException("Quadric is behind camera");
            }
            if (quadric.contains(pose))
            {
                throw QuadricProjectionException("Camera is inside quadric");
            }

            gtsam::Vector3 radii = quadric.radii();
            SemanticTable semantic_table;

            Eigen::Matrix<double, 2, 1> r0;
//        double lenth = radii[0];
//        double width = radii[1];
//        double height = radii[2];
            r0 << radii(0) / radii(2), radii(1) / radii(2);
            auto rs = semantic_table.getEntry(label_);
            if(label_ =="tape"||label_ =="mouse")
            cout<<"SSFactor: "<<label_<<", now ratio: "<<r0<<", antici ratio: "<<rs<<endl;
            gtsam::Vector2 error;
            error << pow((r0 - rs)[0],2) , pow((r0 - rs)[1],2);
//            error = r0 - rs;
//            switch (sigma_scc_) {
//                case 1:
//                    error << (r0 - rs).lpNorm<1>();
//                    break;
//                case 5:
//                    error << (r0 - rs).lpNorm<5>();
//                    break;
//                case 10:
//                    error << (r0 - rs).lpNorm<10>();
//                    break;
//                default:
//                    error << (r0 - rs).lpNorm<1>();
//                    break;}

            if (NUMERICAL_DERIVATIVE)
            {
                std::function<gtsam::Vector(const gtsam::Pose3 &,
                                            const ConstrainedDualQuadric &)>
                        funPtr(boost::bind(&SemanticScaleFactor::evaluateError, this,
                                           boost::placeholders::_1, boost::placeholders::_2,
                                           boost::none, boost::none));
                if (H1)
                {
                    Eigen::Matrix<double, 2, 6> db_dx_ =
                            gtsam::numericalDerivative21(funPtr, pose, quadric, 1e-6);
                    *H1 = db_dx_;
                }
                if (H2)
                {
                    Eigen::Matrix<double, 2, 9> db_dq_ =
                            gtsam::numericalDerivative22(funPtr, pose, quadric, 1e-6);
                    *H2 = db_dq_;
                }
            }

//            std::cout << "Semantic Error: " << error[0]<<"   " <<error[1] <<std::endl;

            return error;

            // check for nans
            if (error.array().isInf().any() || error.array().isNaN().any() ||
                (H1 && (H1->array().isInf().any() || H1->array().isNaN().any())) ||
                (H2 && (H2->array().isInf().any() || H2->array().isNaN().any())))
            {
                throw std::runtime_error("nan/inf error in bbf");
            }
        }
            // handle projection failures
        catch (QuadricProjectionException &e)
        {
            gtsam::Vector2 error = gtsam::Vector2::Ones() * 100000.0;
            if (H1)
            {
                *H1 = gtsam::Matrix::Zero(2, 6);
            }
            if (H2)
            {
                *H2 = gtsam::Matrix::Zero(2, 9);
            }
            return error;
        }
        // Just to avoid warning
        return gtsam::Vector2::Ones() * 1000.0;
    }

    /* ************************************************************************* */
    // the derivative of the error wrt camera pose (1x6)
    gtsam::Matrix SemanticScaleFactor::evaluateH1(
            const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
    {
        gtsam::Matrix H1;
        this->evaluateError(pose, quadric, H1, boost::none);
        return H1;
    }

    /* ************************************************************************* */
    // the derivative of the error wrt quadric (1x9)
    gtsam::Matrix SemanticScaleFactor::evaluateH2(
            const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
    {
        gtsam::Matrix H2;
        this->evaluateError(pose, quadric, boost::none, H2);
        return H2;
    }

    /* ************************************************************************* */
    /** Evaluates the derivative of the error wrt pose */
    gtsam::Matrix SemanticScaleFactor::evaluateH1(const gtsam::Values &x) const
    {
        const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
        const ConstrainedDualQuadric quadric =
                x.at<ConstrainedDualQuadric>(this->objectKey());
        return this->evaluateH1(pose, quadric);
    }

    /* ************************************************************************* */
    /** Evaluates the derivative of the error wrt quadric */
    gtsam::Matrix SemanticScaleFactor::evaluateH2(const gtsam::Values &x) const
    {
        const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
        const ConstrainedDualQuadric quadric =
                x.at<ConstrainedDualQuadric>(this->objectKey());
        return this->evaluateH2(pose, quadric);
    }

    /* ************************************************************************* */
    // Print function
    void SemanticScaleFactor::print(const std::string &s,
                                    const gtsam::KeyFormatter &keyFormatter) const
    {
        cout << s << "SemanticScaleFactor(" << keyFormatter(key1()) << ","
             << keyFormatter(key2()) << ")" << endl;
        cout<<"    Label: " << label_ << endl;
        cout << "    NoiseModel: ";
        noiseModel()->print();
        cout << endl;
    }

    /* ************************************************************************* */
    // Judge function
    bool SemanticScaleFactor::equals(const SemanticScaleFactor &other,
                                     double tol) const
    {
        bool equal = label_ == other.label_ &&
                     calibration_->equals(*other.calibration_, tol) &&
                     noiseModel()->equals(*other.noiseModel(), tol) &&
                     key1() == other.key1() && key2() == other.key2();
        return equal;
    }

} // namespace gtsam_quadrics
