/* ----------------------------------------------------------------------------
@ Compute fbbox @

This is a C++ code implementing the BoundingBoxFactor class in the gtsam_quadrics library.

The BoundingBoxFactor class is used to represent the geometric constraints that the projected quadric bounding box (dual conic) in a camera image should match the given measurements. The measurements are 4D vector, representing the (u,v) position of the upper-left corner of the bounding box and its width and height.

The class includes an evaluateError method that returns the error between the predicted and measured bounding boxes, given a pose and a constrained dual quadric (a quadric in a world frame). It also calculates the corresponding Jacobians if they are requested.

The code also includes a numerical derivative option to calculate the Jacobians. The H1 and H2 arguments of evaluateError represent the Jacobians with respect to the pose and the quadric, respectively. The evaluateH1 and evaluateH2 methods are helper methods that return the corresponding Jacobians without the error vector.

The code uses some classes and functions from the gtsam library, such as Pose3, Matrix, Vector, numericalDerivative21, and numericalDerivative22.
 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 **/

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <gtsam_quadrics/geometry/BoundingBoxFactor.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <boost/bind/bind.hpp> //用于支持函数绑定功能。
// 函数绑定可以让你在定义一个函数对象时，预先绑定部分参数，留下另外一些参数在调用时再指定，从而得到一个类似于函数指针的对象。

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 用于计算一个对象在相机图像平面上的边界框与测量的边界框之间的误差。
  // 这个类的作用是为基于视觉的SLAM算法提供一个约束，以便估计物体在3D空间中的位置和相机的位姿。
  // 函数的输入包括了相机的姿态、表示物体的二次曲面、以及两个可选的矩阵参数H1和H2。
  // 其中，H1和H2分别表示误差函数对相机姿态和二次曲面的导数矩阵，用于后续的优化。
  // Calculate the error between an object bbox on the camera image plane and a measured bbox.
  // This class provides a constraint for vision-based SLAM to estimate the object's position in 3D space and the camera's pose.
  // The function's inputs include: the camera pose, an object quadratic representation, and two optional matrices parameter H1 and H2.
  // H1 and H2 represent the derivative matrices of the error function with respect to the camera pose and the quadratic respectively. 
  // These matrices are used for subsequent optimization.

  // 在函数内部，首先对输入的相机姿态和二次曲面进行一些检查。
  // 如果二次曲面在相机后面或者相机在物体内部，就会抛出一个异常。
  // In the function, the camera pose and quadratic surface inputs are first checked for validity.
  // An exception is thrown if the quadratic is behind the camera or the camera is inside the object.

  // 然后，函数会通过调用QuadricCamera::project()函数将二次曲面投影到相机平面上，并计算相应的导数矩阵。
  // 如果开启了数值导数（NUMERICAL_DERIVATIVE），则会使用数值方法计算导数矩阵。
  // Then, the function projects the quadratic onto the camera plane by calling QuadricCamera::project(),
  // and calculates the corresponding derivative matrices.
  // If NUMERICAL_DERIVATIVE is enabled, numerical methods will be used to calculate the derivative matrices.

  // 接着，函数会检查投影后的二次体是否是一个椭圆，并计算投影后椭圆的边界框及其导数矩阵。
  // Next, the function checks if the projected quadratic is an ellipse and calculates the b-box and its derivative matrices.

  // 最后，函数会计算测量与预测边界框之间的误差，并根据需要计算误差函数对相机姿态和二次曲面的导数矩阵。
  // 如果误差函数或导数矩阵包含无穷大或NaN值，函数会抛出异常。
  // 如果投影出现了问题，函数会通过设置一个固定的误差向量和零导数矩阵来处理异常。
  // Finally, the function calculates the error between the measured and predicted b-boxes and, 
  // if required, the derivative matrices of the error function with respect to the camera pose and the quadratic surface.
  // An exception is thrown if the error function or derivative matrices contain infinite or NaN values.
  // If there are problems with the projection, the function will handle it by setting a fixed error vector and zero derivative matrices.

  // 最后，函数会返回误差向量。
  gtsam::Vector BoundingBoxFactor::evaluateError(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric,
      boost::optional<gtsam::Matrix &> H1,
      boost::optional<gtsam::Matrix &> H2) const
  {
    // 在函数内部，首先对输入的相机姿态和二次曲面进行一些检查。
    // 如果二次曲面在相机后面或者相机在物体内部，就会抛出一个异常。
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

      // project quadric taking into account partial derivatives
      // 定义了两个矩阵 dC_dx 和 dC_dq，分别用于存储双二次曲面 quadric 投影到相机姿态 pose 上的导数。
      // 接着创建了一个 DualConic 对象 dualConic 用于存储 quadric 投影到相机上的双锥体。
      // DualConic 类是一个代表双锥体的类，它是在计算机视觉中广泛使用的表示椭圆形图像边界的数学工具。
      // 在这个函数中，dualConic 将被用于计算图像边界并计算误差函数。
      Eigen::Matrix<double, 9, 6> dC_dx;
      Eigen::Matrix<double, 9, 9> dC_dq;
      DualConic dualConic;

      // 然后，函数会通过调用QuadricCamera::project()函数将二次曲面投影到相机平面上，并计算相应的导数矩阵。
      // 如果开启了数值导数（NUMERICAL_DERIVATIVE），则会使用数值方法计算导数矩阵。
      if (!NUMERICAL_DERIVATIVE)
      {
        dualConic = QuadricCamera::project(quadric, pose, calibration_,
                                           H2 ? &dC_dq : 0, H1 ? &dC_dx : 0);
      }
      else
      {
        dualConic = QuadricCamera::project(quadric, pose, calibration_);
      }

      // check dual conic is valid for error function
      // 接着，函数会检查投影后的二次体是否是一个椭圆。
      if (!dualConic.isEllipse())
      {
        throw QuadricProjectionException("Projected Conic is non-ellipse");
      }

      // calculate conic bounds with derivatives
      // 并计算投影后椭圆的边界框及其导数矩阵
      bool computeJacobians = bool(H1 || H2) && !NUMERICAL_DERIVATIVE;
      Eigen::Matrix<double, 4, 9> db_dC;
      AlignedBox2 predictedBounds;
      if (measurementModel_ == STANDARD)
      {
        predictedBounds = dualConic.bounds(computeJacobians ? &db_dC : 0);
      }
      else if (measurementModel_ == TRUNCATED)
      {
        try
        {
          predictedBounds =
              dualConic.smartBounds(calibration_, computeJacobians ? &db_dC : 0);
        }
        catch (std::runtime_error &e)
        {
          throw QuadricProjectionException("smartbounds failed");
        }
      }

      // evaluate error
      // 最后，函数会计算测量与预测边界框之间的误差。
      gtsam::Vector4 error = predictedBounds.vector() - measured_.vector();
      // 并根据需要计算误差函数对相机姿态和二次曲面的导数矩阵
      if (NUMERICAL_DERIVATIVE)
      {
        std::function<gtsam::Vector(const gtsam::Pose3 &,
                                    const ConstrainedDualQuadric &)>
            funPtr(boost::bind(&BoundingBoxFactor::evaluateError, this,
                               boost::placeholders::_1, boost::placeholders::_2,
                               boost::none, boost::none));
        if (H1)
        {
          Eigen::Matrix<double, 4, 6> db_dx_ =
              gtsam::numericalDerivative21(funPtr, pose, quadric, 1e-6);
          *H1 = db_dx_;
        }
        if (H2)
        {
          Eigen::Matrix<double, 4, 9> db_dq_ =
              gtsam::numericalDerivative22(funPtr, pose, quadric, 1e-6);
          *H2 = db_dq_;
        }
      }
      else
      {
        // calculate derivative of error wrt pose
        if (H1)
        {
          // combine partial derivatives
          *H1 = db_dC * dC_dx;
        }

        // calculate derivative of error wrt quadric
        if (H2)
        {
          // combine partial derivatives
          *H2 = db_dC * dC_dq;
        }
      }

      return error;

      // check for nans
      // 如果误差函数或导数矩阵包含无穷大或NaN值，函数会抛出异常。
      if (error.array().isInf().any() || error.array().isNaN().any() ||
          (H1 && (H1->array().isInf().any() || H1->array().isNaN().any())) ||
          (H2 && (H2->array().isInf().any() || H2->array().isNaN().any())))
      {
        throw std::runtime_error("nan/inf error in bbf");
      }
    }
    // handle projection failures
    // 如果投影出现了问题，函数会通过设置一个固定的误差向量和零导数矩阵来处理异常。。
    catch (QuadricProjectionException &e)
    {
      // std::cout << "  Landmark " << symbolIndex(this->objectKey()) << "
      // received: " << e.what() << std::endl;

      // if error cannot be calculated
      // set error vector and jacobians to zero
      gtsam::Vector4 error = gtsam::Vector4::Ones() * 1000.0;
      if (H1)
      {
        *H1 = gtsam::Matrix::Zero(4, 6);
      }
      if (H2)
      {
        *H2 = gtsam::Matrix::Zero(4, 9);
      }
      // 最后，函数会返回误差向量
      return error;
    }
  }

  /* ************************************************************************* */
  // the derivative of the error wrt camera pose (4x6)
  // 这个函数计算误差函数对相机姿态的雅可比矩阵H1。
  // 它的输入是一个位姿和一个受限对偶二次曲面（ConstrainedDualQuadric），输出是一个4×6的雅可比矩阵。
  // 这个函数首先声明一个空的矩阵H1，计算误差函数对相机姿态的导数矩阵，并将结果存储在H1中。
  gtsam::Matrix BoundingBoxFactor::evaluateH1(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
  {
    gtsam::Matrix H1;
    this->evaluateError(pose, quadric, H1, boost::none);
    return H1;
  }

  /* ************************************************************************* */
  // the derivative of the error wrt quadric (4x9)
  // 这个函数计算误差函数对二次曲面的雅可比矩阵H2。
  // 它的输入是一个位姿和一个受限对偶二次曲面（ConstrainedDualQuadric），输出是一个4×9的雅可比矩阵。
  // 这个函数首先声明一个空的矩阵H2，计算误差函数对二次曲面的导数矩阵，并将结果存储在H2中。
  gtsam::Matrix BoundingBoxFactor::evaluateH2(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
  {
    gtsam::Matrix H2;
    this->evaluateError(pose, quadric, boost::none, H2);
    return H2;
  }

  /* ************************************************************************* */
  // 这个函数计算误差函数对相机姿态的雅可比矩阵H1。
  // 但它的输入是一个gtsam::Values对象，其中包含了该因子的位姿和受限对偶二次曲面的值。
  // 它通过从x中提取位姿和对象的值，并将它们传递给evaluateH1函数来计算误差函数对相机姿态的雅可比矩阵H1。
  gtsam::Matrix BoundingBoxFactor::evaluateH1(const gtsam::Values &x) const
  {
    const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
    const ConstrainedDualQuadric quadric =
        x.at<ConstrainedDualQuadric>(this->objectKey());
    return this->evaluateH1(pose, quadric);
  }

  /* ************************************************************************* */
  // 这个函数计算误差函数对二次曲面的雅可比矩阵H2。
  // 但它的输入是一个gtsam::Values对象，其中包含了该因子的位姿和受限对偶二次曲面的值。
  // 它通过从x中提取位姿和对象的值，并将它们传递给evaluateH2函数来计算误差函数对相机姿态的雅可比矩阵H2。
  gtsam::Matrix BoundingBoxFactor::evaluateH2(const gtsam::Values &x) const
  {
    const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
    const ConstrainedDualQuadric quadric =
        x.at<ConstrainedDualQuadric>(this->objectKey());
    return this->evaluateH2(pose, quadric);
  }

  /* ************************************************************************* */
  // 这个函数用于打印BoundingBoxFactor的信息。它输出该因子的关键字、测量值和噪声模型。
  void BoundingBoxFactor::print(const std::string &s,
                                const gtsam::KeyFormatter &keyFormatter) const
  {
    cout << s << "BoundingBoxFactor(" << keyFormatter(key1()) << ","
         << keyFormatter(key2()) << ")" << endl;
    measured_.print("    Measured: ");
    cout << "    NoiseModel: ";
    noiseModel()->print();
    cout << endl;
  }

  /* ************************************************************************* */
  // 用于检查当前的BoundingBoxFactor对象是否与另一个BoundingBoxFactor对象other相等。
  // 函数内部调用了measured_、calibration_和noiseModel()对象的equals方法来检查这三个对象是否分别相等。
  // 其中measured_表示测量值，calibration_表示相机标定信息，noiseModel()表示误差模型。
  // 如果它们都相等，则检查当前对象的两个关键字是否与other对象的关键字相同。
  bool BoundingBoxFactor::equals(const BoundingBoxFactor &other,
                                 double tol) const
  {
    bool equal = measured_.equals(other.measured_, tol) &&
                 calibration_->equals(*other.calibration_, tol) &&
                 noiseModel()->equals(*other.noiseModel(), tol) &&
                 key1() == other.key1() && key2() == other.key2();
    return equal;
  }

} // namespace gtsam_quadrics
