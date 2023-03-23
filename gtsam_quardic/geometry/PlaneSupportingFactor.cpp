/**
 * @file PlaneSupportingFactor.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */


#define NUMERICAL_DERIVATIVE ture

using namespace std;

namespace gtsam_quadrics 
{

gtsam::Vector PlaneSupportingFactor::evaluateError(
    const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric,
    boost::optional<gtsam::Matrix &> H1,
    boost::optional<gtsam::Matrix &> H2) const 
{
  
  try 
  {
    if (quadric.isBehind(pose)) 
    {
      throw PlaneSupportingException("Quadric is behind camera");
    }
    if (quadric.contains(pose)) 
    {
      throw PlaneSupportingException("Camera is inside quadric");
    }

  
    gtsam::Vector3 radius = quadric.radii();
    gtsam::Matrix44 normalized_Q = quadric.normalizedMatrix();
    Eigen::MatrixXd matrix(normalized_Q.rows(), normalized_Q.cols());
    for (int i = 0; i < normalized_Q.rows(); i++) 
    {
      for (int j = 0; j < normalized_Q.cols(); j++) 
      {
        matrix(i,j) = normalized_Q(i,j);
      }
    }
    Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
    Eigen::VectorXd eigenvalues = eigensolver.eigenvalues().real();
    Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors().real();

    /* Find the eigen value for x */ 
    double desired_eigenvalue_x = 1 / (-radius.x() ** 2); // for example
    int i = 0;
    for (i = 0; i < eigenvalues.size(); i++) 
    {
      if (std::abs(eigenvalues(i) - desired_eigenvalue_x) < 1e-2) 
      {
        break; // found the eigenvalue
      }
    }
    if (i == eigenvalues.size()) 
    {
      std::cout << "Desired eigenvalue not found." << std::endl;
    } 
    else {
    // Get the eigenvector corresponding to the desired eigenvalue
    Eigen::VectorXd desired_eigenvector_x = eigenvectors.col(i);
    std::cout << "Eigenvector corresponding to eigenvalue " << desired_eigenvalue_x << ":" << std::endl;
    std::cout << desired_eigenvector_x << std::endl;
    }

    /* find the eigen value for y */
    double desired_eigenvalue_y = 1 / (-radius.y() ** 2); // for example
    for (i = 0; i < eigenvalues.size(); i++) 
    {
      if (std::abs(eigenvalues(i) - desired_eigenvalue_y) < 1e-2) 
      {
        break; // found the eigenvalue
      }
    }

    if (i == eigenvalues.size()) 
    {
      std::cout << "Desired eigenvalue not found." << std::endl;
    } 
    else 
    {
    // Get the eigenvector corresponding to the desired eigenvalue
    Eigen::VectorXd desired_eigenvector_y = eigenvectors.col(i);
    std::cout << "Eigenvector corresponding to eigenvalue " << desired_eigenvalue_y << ":" << std::endl;
    std::cout << desired_eigenvector_y << std::endl;
    }
    
    /* convert the eigen format to gtsam format */ 
    gtsam::Vector4 normal_x = gtsam::Vector4::fromEigen(desired_eigenvector_x);
    gtsam::Vector4 normal_y = gtsam::Vector4::fromEigen(desired_eigenvector_y);  

    AlignedBox3 constrainBox = quadric.bounds();
    gtsam::vector4 support_plane_normal << 0, 0, 1, constrainBox.zmin();

    gtsam::Vector1 error_tagent  = support_plane_normal.transpose() * normalized_Q * support_plane_normal;
    gtsam::Vector1 error_xNormal = normal_x.transpose() * support_plane_normal;
    gtsam::Vector1 error_yNormal = normal_y.transpose() * support_plane_normal;

    gtsam::Vector1 totalError = error_tagent + error_xNormal + error_yNormal;

    return totalError;
  

    if (NUMERICAL_DERIVATIVE) 
    {
      std::function<gtsam::Vector(const gtsam::Pose3 &,
                                  const ConstrainedDualQuadric &)>
          funPtr(boost::bind(&SemanticScaleFactor::evaluateError, this,
                              boost::placeholders::_1, boost::placeholders::_2,
                              boost::none, boost::none));
      if (H1) 
      {
        Eigen::Matrix<double, 1, 6> db_dx_ =
          gtsam::numericalDerivative21(funPtr, pose, quadric, 1e-6);
        *H1 = db_dx_;
      }
      if (H2) 
      {
        Eigen::Matrix<double, 1, 9> db_dq_ =
          gtsam::numericalDerivative22(funPtr, pose, quadric, 1e-6);
        *H2 = db_dq_;
      }
    }

    return totalError;

    // check for nans
    // 如果误差函数或导数矩阵包含无穷大或NaN值，函数会抛出异常。
    if (totalError.array().isInf().any() || totalError.array().isNaN().any() ||
        (H1 && (H1->array().isInf().any() || H1->array().isNaN().any())) ||
        (H2 && (H2->array().isInf().any() || H2->array().isNaN().any()))) 
    {
        throw std::runtime_error("nan/inf error in bbf");
    }
  }


  catch (PlaneSupportingException &e)
  {
    // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
    // received: " << e.what() << std::endl;

    // if error cannot be calculated
    // set error vector and jacobians to zero
    gtsam::Vector1 totalError = gtsam::Vector1::Ones() * 1000.0;
    if (H1)
    {
      *H1 = gtsam::Matrix::Zero(1, 6);
    }
    if (H2)
    {
      *H2 = gtsam::Matrix::Zero(1, 9);
    }
    return totalError;
  }
}



gtsam::Matrix PlaneSupportingFactor::evaluateH1(
    const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
{
  gtsam::Matrix H1;
  this->evaluateError(pose, quadric, H1, boost::none);
  return H1;
}

gtsam::Matrix PlaneSupportingFactor::evaluateH2(
    const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
{
  gtsam::Matrix H2;
  this->evaluateError(pose, quadric, boost::none, H2);
  return H2;
}

gtsam::Matrix PlaneSupportingFactor::evaluateH1(const gtsam::Values &x) const
{
  const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
  const ConstrainedDualQuadric quadric =
      x.at<ConstrainedDualQuadric>(this->objectKey());
  return this->evaluateH1(pose, quadric);
}

gtsam::Matrix PlaneSupportingFactor::evaluateH2(const gtsam::Values &x) const
{
  const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
  const ConstrainedDualQuadric quadric =
      x.at<ConstrainedDualQuadric>(this->objectKey());
  return this->evaluateH2(pose, quadric);
}

void PlaneSupportingFactor::print(const std::string& s, 
                                  const gtsam::KeyFormatter& keyFormatter) const
{
    cout << s << "PlaneSupportingFactor(" << keyFormatter(key()) << ")" << endl;
    cout << "    NoiseModel: "; 
    noiseModel()->print(); 
    cout << endl;
}

};