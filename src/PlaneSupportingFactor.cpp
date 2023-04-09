#include "QuadricProjectionException.h"
#include "PlaneSupportingFactor.h"
#include "SemanticTable.h"

#include <gtsam/base/numericalDerivative.h>
#include <boost/bind/bind.hpp>
#include <Eigen/Dense>
#include <iostream>

#define NUMERICAL_DERIVATIVE true

using namespace std;

namespace gtsam_soslam
{
  gtsam::Vector PlaneSupportingFactor::evaluateError(
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

      // ====================================================================
//      gtsam::Vector3 radius = quadric.radii();
      gtsam::Matrix33 rotational = quadric.pose().rotation().matrix();
//      gtsam::Matrix44 normalized_Q = quadric.normalizedMatrix();


      gtsam::Vector3 x_axis(1, 0, 0), y_axis(0, 1, 0);
      x_axis = rotational * x_axis;
      y_axis = rotational * y_axis;

      gtsam::Vector2 error;
      error << 1000 * x_axis[2], 1000 *  y_axis[2];
      cout << "x error: " << error(0) << ", y error: " << error(1) << endl;

//      Eigen::MatrixXd matrix(normalized_Q.rows(), normalized_Q.cols());

//      for (int i = 0; i < normalized_Q.rows(); i++)
//      {
//        for (int j = 0; j < normalized_Q.cols(); j++)
//        {
//          matrix(i, j) = normalized_Q(i, j);
//        }
//      }

//      Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
//      Eigen::VectorXd eigenvalues = eigensolver.eigenvalues().real();
//      Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors().real();
//
//      /* Find the eigen value for x */
//      double desired_eigenvalue_x = 1 / std::pow(-radius.x(), 2); // for example
//      int i = 0;
//      for (i = 0; i < eigenvalues.size(); i++)
//      {
//        if (std::abs(eigenvalues(i) - desired_eigenvalue_x) < 1e-2)
//        {
//          break; // found the eigenvalue
//        }
//      }
//
//      Eigen::VectorXd desired_eigenvector_x;
//      if (i == eigenvalues.size())
//      {
////        std::cout << "Desired eigenvalue not found." << std::endl;
//      }
//      else
//      {
//        // Get the eigenvector corresponding to the desired eigenvalue
//        desired_eigenvector_x = eigenvectors.col(i);
////        std::cout << "Eigenvector corresponding to eigenvalue " << desired_eigenvalue_x << ":" << std::endl;
////        std::cout << "des x: " << desired_eigenvector_x << std::endl;
//      }
//
//      /* find the eigen value for y */
//      double desired_eigenvalue_y = 1 / std::pow(-radius.y(), 2); // for example
//      for (i = 0; i < eigenvalues.size(); i++)
//      {
//        if (std::abs(eigenvalues(i) - desired_eigenvalue_y) < 1e-2)
//        {
//          break; // found the eigenvalue
//        }
//      }
//
//      Eigen::VectorXd desired_eigenvector_y;
//      if (i == eigenvalues.size())
//      {
////        std::cout << "Desired eigenvalue not found." << std::endl;
//      }
//      else
//      {
//        // Get the eigenvector corresponding to the desired eigenvalue
//        desired_eigenvector_y = eigenvectors.col(i);
////        std::cout << "Eigenvector corresponding to eigenvalue " << desired_eigenvalue_y << ":" << std::endl;
////        std::cout << "des y: " << desired_eigenvector_y << std::endl;
//      }

      /* convert the eigen format to gtsam format */
      // gtsam::Vector4 normal_x = gtsam::Vector4::fromEigen(desired_eigenvector_x);
//      std::cout << "des x size: " << desired_eigenvector_x.size() << std::endl;
//      std::cout << "des y size: " << desired_eigenvector_y.size() << std::endl;
//      gtsam::Vector3 error;
//      if (desired_eigenvector_x.size() == 4 && desired_eigenvector_y.size() == 4)
//      {
//        gtsam::Vector4 normal_x(desired_eigenvector_x(0), desired_eigenvector_x(1), desired_eigenvector_x(2), desired_eigenvector_x(3));
//
//        // gtsam::Vector4 normal_y = gtsam::Vector4::fromEigen(desired_eigenvector_y);
//        gtsam::Vector4 normal_y(desired_eigenvector_y(0), desired_eigenvector_y(1), desired_eigenvector_y(2), desired_eigenvector_y(3));
//
//        AlignedBox3 constrainBox = quadric.bounds();
//        gtsam::Vector4 support_plane_normal(0, 0, 1, constrainBox.zmin());
//
//        gtsam::Vector1 error_tagent = support_plane_normal.transpose() * normalized_Q * support_plane_normal;
//        gtsam::Vector1 error_xNormal = normal_x.transpose() * support_plane_normal;
//        gtsam::Vector1 error_yNormal = normal_y.transpose() * support_plane_normal;
//
//        error << error_tagent , error_xNormal , error_yNormal;
//      }
//      else
//      {
//        double large_error = 5;
//        error(0) = large_error;
//      }

      // cout << "error: " << error << endl;

      if (NUMERICAL_DERIVATIVE)
      {
        std::function<gtsam::Vector(const gtsam::Pose3 &,
                                    const ConstrainedDualQuadric &)>
            funPtr(boost::bind(&PlaneSupportingFactor::evaluateError, this,
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
        std::cout << "PSC Error: " << error[0] <<std::endl;

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
    return gtsam::Vector2::Ones() * 100000.0;
  }

  /* ************************************************************************* */
  // the derivative of the error wrt camera pose (2x6)
  gtsam::Matrix PlaneSupportingFactor::evaluateH1(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
  {
    gtsam::Matrix H1;
    this->evaluateError(pose, quadric, H1, boost::none);
    return H1;
  }

  /* ************************************************************************* */
  // the derivative of the error wrt quadric (2x9)
  gtsam::Matrix PlaneSupportingFactor::evaluateH2(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
  {
    gtsam::Matrix H2;
    this->evaluateError(pose, quadric, boost::none, H2);
    return H2;
  }

  /* ************************************************************************* */
  /** Evaluates the derivative of the error wrt pose */
  gtsam::Matrix PlaneSupportingFactor::evaluateH1(const gtsam::Values &x) const
  {
    const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
    const ConstrainedDualQuadric quadric =
        x.at<ConstrainedDualQuadric>(this->objectKey());
    return this->evaluateH1(pose, quadric);
  }

  /* ************************************************************************* */
  /** Evaluates the derivative of the error wrt quadric */
  gtsam::Matrix PlaneSupportingFactor::evaluateH2(const gtsam::Values &x) const
  {
    const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
    const ConstrainedDualQuadric quadric =
        x.at<ConstrainedDualQuadric>(this->objectKey());
    return this->evaluateH2(pose, quadric);
  }

  /* ************************************************************************* */
  // Print function
  void PlaneSupportingFactor::print(const std::string &s,
                                    const gtsam::KeyFormatter &keyFormatter) const
  {
    cout << s << "PlaneSupportingFactor(" << keyFormatter(key1()) << ","
         << keyFormatter(key2()) << ")" << endl;
    cout << "    Label: " << label_ << endl;
    cout << "    NoiseModel: ";
    noiseModel()->print();
    cout << endl;
  }

  /* ************************************************************************* */
  // Judge function
  bool PlaneSupportingFactor::equals(const PlaneSupportingFactor &other,
                                     double tol) const
  {
    bool equal = label_ == other.label_ &&
                 calibration_->equals(*other.calibration_, tol) &&
                 noiseModel()->equals(*other.noiseModel(), tol) &&
                 key1() == other.key1() && key2() == other.key2();
    return equal;
  }

} // namespace gtsam_quadrics
