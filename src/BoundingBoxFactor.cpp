/**
 * @file BoundingBoxFactor.cpp
 * @author Lachlan Nicholson
 * @modified_by ziqi han
 * @modified_date 5/04/23
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#include "QuadricProjectionException.h"
#include "BoundingBoxFactor.h"
#include "QuadricCamera.h"

#include <eigen3/Eigen/Dense>
#include <boost/bind/bind.hpp>
#include <gtsam/base/numericalDerivative.h>

#define NUMERICAL_DERIVATIVE true

using namespace std;

namespace gtsam_soslam {

// rewrite w.r.t SOSLAM paper
gtsam::Vector BoundingBoxFactor::evaluateError(
    const gtsam::Pose3& pose, const ConstrainedDualQuadric& quadric,
    boost::optional<gtsam::Matrix&> H1,
    boost::optional<gtsam::Matrix&> H2) const {
    try {
    // check pose-quadric pair
    if (quadric.isBehind(pose)) {
      throw QuadricProjectionException("Quadric is behind camera");
    }
    if (quadric.contains(pose)) {
      throw QuadricProjectionException("Camera is inside quadric");
    }
    gtsam::Vector1 error = gtsam::Vector1::Zero();

    std::vector<gtsam::Vector4> planes = QuadricCamera::project(measured_, pose, calibration_);

    switch (sigma_bbs_) {
      case 1:
          for (auto plane : planes){
              gtsam::Vector1 temp_error((plane.transpose() * quadric.matrix() * plane).lpNorm<1>());
              error = error + temp_error;
          }
          break;
      case 5:
          for (auto plane : planes){
              gtsam::Vector1 temp_error((plane.transpose() * quadric.matrix() * plane).lpNorm<5>());
              error = error + temp_error;
          }
          break;
      case 10:
          for (auto plane : planes){
              gtsam::Vector1 temp_error((plane.transpose() * quadric.matrix() * plane).lpNorm<10>());
              error = error + temp_error;
          }
          break;
      default:
          for (auto plane : planes){
              gtsam::Vector1 temp_error((plane.transpose() * quadric.matrix() * plane).lpNorm<10>());
              error = error + temp_error;
          }
          break;}

    if (NUMERICAL_DERIVATIVE) {
      std::function<gtsam::Vector(const gtsam::Pose3&,
                                  const ConstrainedDualQuadric&)>
          funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this,
                             boost::placeholders::_1, boost::placeholders::_2,
                             boost::none, boost::none));
      if (H1) {
        Eigen::Matrix<double, 1, 6> db_dx_ =
            gtsam::numericalDerivative21(funPtr, pose, quadric, 1e-6);
        *H1 = db_dx_;
      }
      if (H2) {
        Eigen::Matrix<double, 1, 9> db_dq_ =
            gtsam::numericalDerivative22(funPtr, pose, quadric, 1e-6);
        *H2 = db_dq_;
      }
    }
//    std::cout << "BBC Error: " << error / 10000 <<std::endl;
    return error / 10000;

    // check for nans
    if (error.array().isInf().any() || error.array().isNaN().any() ||
        (H1 && (H1->array().isInf().any() || H1->array().isNaN().any())) ||
        (H2 && (H2->array().isInf().any() || H2->array().isNaN().any()))) {
      throw std::runtime_error("nan/inf error in bbf");
    }

    // handle projection failures
  } catch (QuadricProjectionException& e) {
        // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
        // received: " << e.what() << std::endl;

        // if error cannot be calculated
        // set error vector and jacobians to zero
        gtsam::Vector1 error = gtsam::Vector1::Ones() * 1000.0;
        if (H1) {
          *H1 = gtsam::Matrix::Zero(1, 6);
        }
        if (H2) {
          *H2 = gtsam::Matrix::Zero(1, 9);
        }

        return error;
    }
    // Just to avoid warning
    return gtsam::Vector1::Ones() * 1000.0;
}

/* ************************************************************************* */
gtsam::Matrix BoundingBoxFactor::evaluateH1(
    const gtsam::Pose3& pose, const ConstrainedDualQuadric& quadric) const {
  gtsam::Matrix H1;
  this->evaluateError(pose, quadric, H1, boost::none);
  return H1;
}

/* ************************************************************************* */
gtsam::Matrix BoundingBoxFactor::evaluateH2(
    const gtsam::Pose3& pose, const ConstrainedDualQuadric& quadric) const {
  gtsam::Matrix H2;
  this->evaluateError(pose, quadric, boost::none, H2);
  return H2;
}

/* ************************************************************************* */
gtsam::Matrix BoundingBoxFactor::evaluateH1(const gtsam::Values& x) const {
  const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
  const ConstrainedDualQuadric quadric =
      x.at<ConstrainedDualQuadric>(this->objectKey());
  return this->evaluateH1(pose, quadric);
}

/* ************************************************************************* */
gtsam::Matrix BoundingBoxFactor::evaluateH2(const gtsam::Values& x) const {
  const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
  const ConstrainedDualQuadric quadric =
      x.at<ConstrainedDualQuadric>(this->objectKey());
  return this->evaluateH2(pose, quadric);
}

/* ************************************************************************* */
void BoundingBoxFactor::print(const std::string& s,
                              const gtsam::KeyFormatter& keyFormatter) const {
  cout << s << "BoundingBoxFactor(" << keyFormatter(key1()) << ","
       << keyFormatter(key2()) << ")" << endl;
  measured_.print("    Measured: ");
  cout << "    NoiseModel: ";
  noiseModel()->print();
  cout << endl;
}

/* ************************************************************************* */
bool BoundingBoxFactor::equals(const BoundingBoxFactor& other,
                               double tol) const {
  bool equal = measured_.equals(other.measured_, tol) &&
               calibration_->equals(*other.calibration_, tol) &&
               noiseModel()->equals(*other.noiseModel(), tol) &&
               key1() == other.key1() && key2() == other.key2();
  return equal;
}

}  // namespace gtsam_soslam
