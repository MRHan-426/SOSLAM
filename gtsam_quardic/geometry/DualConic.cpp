/* ----------------------------------------------------------------------------
This is a C++ code that defines some functions for the DualConic class, which is a class for representing dual conics in the projective plane. Here is a summary of the functions:
平面对偶锥曲线的生成。
@ 核心函数：AlignedBox2 DualConic::smartBounds，实现了论文的计算式（4），即Figure2中的右图。@

DualConic::DualConic():
  Default constructor that initializes the dual conic as a unit circle centered at the origin.
DualConic::DualConic(const gtsam::Pose2& pose, const gtsam::Vector2& radii):
  Constructor that initializes the dual conic using a pose (position and orientation) and a vector of radii.
DualConic DualConic::normalize(void) const:
  Returns a new dual conic that is a normalized version of the current one.
AlignedBox2 DualConic::bounds(gtsam::OptionalJacobian<4, 9> H) const:
  Returns the bounding box of the dual conic, which is an AlignedBox2 object that contains the minimum and maximum x and y coordinates of the conic. The function also optionally computes the Jacobian of the bounding box with respect to the parameters of the dual conic.
 * -------------------------------------------------------------------------- */

/**
 * @file DualConic.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a dual conic
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/QuadricProjectionException.h>
#include <gtsam_quadrics/base/Utilities.h>
#include <gtsam_quadrics/geometry/DualConic.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <cmath>
#include <iomanip>
#include <iostream>

using namespace std;

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 这个函数是DualConic类的构造函数。当创建一个DualConic对象时，如果没有给出参数，将调用这个构造函数。
  // 它创建了一个初始的对偶锥曲线，可以表示为方程x^2+y^2-z^2=0。
  // 在这里，dC_是一个3x3的矩阵，代表着对偶锥曲线的系数矩阵。
  DualConic::DualConic()
  {
    dC_ = (gtsam::Matrix33() << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, -1.0)
              .finished();
  }

  /* ************************************************************************* */
  // 这个函数是DualConic类的构造函数。当创建一个DualConic对象时，
  // 如果给出一个gtsam::Pose2类型的姿态和一个gtsam::Vector2类型的半径，则将调用这个构造函数。
  // 它使用提供的姿态和半径创建一个新的对偶锥曲线对象。
  // 在这个构造函数中，我们先计算半径矩阵Cc，然后使用它和姿态矩阵Z计算系数矩阵dC_。
  DualConic::DualConic(const gtsam::Pose2 &pose, const gtsam::Vector2 &radii)
  {
    gtsam::Matrix33 Z = pose.matrix();
    gtsam::Matrix33 Cc = (gtsam::Vector3() << (radii).array().pow(2), -1.0)
                             .finished()
                             .asDiagonal();
    gtsam::Matrix33 dC = Z * Cc * Z.transpose();
    dC_ = dC;
  }

  /* ************************************************************************* */
  // 归一化DualConic对象。在归一化的对偶锥曲线中，系数矩阵的最后一个元素是1。
  DualConic DualConic::normalize(void) const
  {
    return DualConic(dC_ / dC_(2, 2));
  }

  /* ************************************************************************* */
  // 该函数计算对偶锥曲线的边界框（即矩形范围）。
  // 可以选择计算边界框相对于对偶锥曲线矩阵参数的Jacobian矩阵。
  // 需要在这里添加代码来确保对偶锥曲线的合法性
  /// TODO: assert conic is closed (eccentricity) // 断言对偶圆锥是闭合的（即离心率存在）
  /// assert bounds are real-valued // 断言边界是实值的
  /// normalize conic // 规范化对偶圆锥。

  // 函数使用对偶锥曲线的矩阵参数计算边界框的四个角（xmin，ymin，xmax，ymax）。
  AlignedBox2 DualConic::bounds(gtsam::OptionalJacobian<4, 9> H) const
  {
    double xmin =
        (dC_(0, 2) + sqrt(dC_(0, 2) * dC_(0, 2) - dC_(2, 2) * dC_(0, 0))) /
        dC_(2, 2);
    double xmax =
        (dC_(0, 2) - sqrt(dC_(0, 2) * dC_(0, 2) - dC_(2, 2) * dC_(0, 0))) /
        dC_(2, 2);
    double ymin =
        (dC_(1, 2) + sqrt(dC_(1, 2) * dC_(1, 2) - dC_(2, 2) * dC_(1, 1))) /
        dC_(2, 2);
    double ymax =
        (dC_(1, 2) - sqrt(dC_(1, 2) * dC_(1, 2) - dC_(2, 2) * dC_(1, 1))) /
        dC_(2, 2);

    // 如果指定了可选参数Jacobian，则函数还将计算边界框相对于对偶锥曲线矩阵参数的Jacobian矩阵。
    if (H)
    {
      Eigen::Matrix<double, 4, 9> db_dC = gtsam::Matrix::Zero(4, 9);
      double f = sqrt(dC_(0, 2) * dC_(0, 2) - dC_(0, 0) * dC_(2, 2));
      double g = sqrt(dC_(1, 2) * dC_(1, 2) - dC_(1, 1) * dC_(2, 2));
      db_dC(0, 0) = 1.0 / f * (-1.0 / 2.0);
      db_dC(0, 6) = (dC_(0, 2) * 1.0 / f + 1.0) / dC_(2, 2);
      db_dC(0, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(0, 2) + f) -
                    (dC_(0, 0) * 1.0 / f * (1.0 / 2.0)) / dC_(2, 2);
      db_dC(1, 4) = 1.0 / g * (-1.0 / 2.0);
      db_dC(1, 7) = (dC_(1, 2) * 1.0 / g + 1.0) / dC_(2, 2);
      db_dC(1, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(1, 2) + g) -
                    (dC_(1, 1) * 1.0 / g * (1.0 / 2.0)) / dC_(2, 2);
      db_dC(2, 0) = 1.0 / f * (1.0 / 2.0);
      db_dC(2, 6) = -(dC_(0, 2) * 1.0 / f - 1.0) / dC_(2, 2);
      db_dC(2, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(0, 2) - f) +
                    (dC_(0, 0) * 1.0 / f * (1.0 / 2.0)) / dC_(2, 2);
      db_dC(3, 4) = 1.0 / g * (1.0 / 2.0);
      db_dC(3, 7) = -(dC_(1, 2) * 1.0 / g - 1.0) / dC_(2, 2);
      db_dC(3, 8) = -1.0 / (dC_(2, 2) * dC_(2, 2)) * (dC_(1, 2) - g) +
                    (dC_(1, 1) * 1.0 / g * (1.0 / 2.0)) / dC_(2, 2);
      *H = db_dC;
    }

    return AlignedBox2(xmin, ymin, xmax, ymax);
  }

  /* ************************************************************************* */
  // @ 论文中的计算式（4） @
  // 这是一个计算对偶锥曲线（dual conic）边界（bounds）的函数，以适当的形式返回一个AlignedBox2类型的边界。
  // 对偶锥曲线是二次曲线，可以通过投影相机的内部参数矩阵和一个四元素描述的相机姿态（即旋转和平移）来计算。
  AlignedBox2 DualConic::smartBounds(
      const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
      gtsam::OptionalJacobian<4, 9> H) const
  {
    // 通过输入的内部参数矩阵计算图像的宽度和高度，生成一个图像边界AlignedBox2类型的对象。
    double imageWidth = 640.0;  // double imageWidth = calibration->px() * 2.0;
    double imageHeight = 480.0; // double imageHeight = calibration->py() * 2.0;
    AlignedBox2 imageBounds(0.0, 0.0, imageWidth, imageHeight);

    // 如果对偶锥曲线完全在图像内部，则使用函数dualconic::bounds()计算简单的边界（一个轴对齐的矩形）。
    Eigen::Matrix<double, 4, 9> simpleJacobian;
    AlignedBox2 simpleBounds = this->bounds(H ? &simpleJacobian : 0);
    if (imageBounds.contains(simpleBounds))
    {
      if (H)
      {
        *H = simpleJacobian;
      }
      return simpleBounds;
    }

    // ensure quadric is at least partially visible
    // NOTE: this will not work because bounds can be inside whilst conic is
    // completely outside imagebounds if (!imageBounds.contains(simpleBounds) &&
    // !imageBounds.intersects(simpleBounds)) {
    //   throw QuadricProjectionException("SimpleBounds outside ImageBounds,
    //   implies quadric not visible");
    // }

    // 如果对偶锥曲线不完全在图像内部，则需要计算更复杂的边界。
    // 首先将对偶锥曲线转换为长双精度（long double）形式，以避免出现负数，然后计算其逆矩阵C并归一化。
    Eigen::Matrix<long double, 3, 3> dualConic = dC_.cast<long double>();
    dualConic = dualConic / dualConic(2, 2);

    // calculate point conic
    Eigen::Matrix<long double, 3, 3> C = dualConic.inverse();

    // normalize conic so polynomials behave
    C = C / C(2, 2);

    std::vector<gtsam::Point2> points;

    try
    {
      // 然后通过解决二次方程组计算点和圆锥的交点，以计算复杂边界的顶点。
      // 可以通过从C的特征向量中提取的信息来计算二次方程组。
      // solve intersection of dC/dx and conic C (solving y values first)
      gtsam::Vector2 ys = utils::solvePolynomial(
          C(1, 1) - pow(C(1, 0) * 2, 2) / (4 * C(0, 0)),
          C(2, 1) * 2 - (C(1, 0) * 2 * C(2, 0) * 2) / (2 * C(0, 0)),
          C(2, 2) - pow(C(2, 0) * 2, 2) / (4 * C(0, 0)));
      gtsam::Point2 p1((-C(1, 0) * 2 * ys[0] - C(2, 0) * 2) / (2 * C(0, 0)),
                       ys[0]);
      gtsam::Point2 p3((-C(1, 0) * 2 * ys[1] - C(2, 0) * 2) / (2 * C(0, 0)),
                       ys[1]);

      // solve intersection of dC/dx and conic C (solving x values first)
      // Vector2 xs = gtsam::utils::solvePolynomial(
      //   4*pow(C(0,0),2)*C(1,1)/pow(C(1,0)*2,2) - C(0,0),
      //   4*C(0,0)*C(1,1)*C(2,0)*2/pow(C(1,0)*2,2) - 2*C(0,0)*C(2,1)*2/C(1,0)*2,
      //   C(1,1)*pow(C(2,0)*2,2)/pow(C(1,0)*2,2) - C(2,0)*2*C(2,1)*2/C(1,0)*2 +
      //   C(2,2)
      // );

      // solve intersection of dC/dy and conic C (solving y values first)
      // Vector2 ys = gtsam::utils::solvePolynomial(
      //   4*C(0,0)*pow(C(1,1),2)/pow(C(1,0)*2,2) - C(1,1),
      //   4*C(0,0)*C(1,1)*C(2,1)*2/pow(C(1,0)*2,2) -
      //   (2*C(1,1)*C(2,0)*2/C(1,0)*2), C(0,0)*pow(C(2,1)*2,2)/pow(C(1,0)*2,2) -
      //   (C(2,0)*2*C(2,1)*2/C(1,0)*2) + C(2,2)
      // );

      // solve intersection of dC/dy and conic C (solving x values first)
      gtsam::Vector2 xs = utils::solvePolynomial(
          C(0, 0) - pow(C(1, 0) * 2, 2) / (4 * C(1, 1)),
          C(2, 0) * 2 - (C(1, 0) * 2 * C(2, 1) * 2) / (2 * C(1, 1)),
          C(2, 2) - pow(C(2, 1) * 2, 2) / (4 * C(1, 1)));
      gtsam::Point2 p0(xs[0],
                       (-C(1, 0) * 2 * xs[0] - C(2, 1) * 2) / (2 * C(1, 1)));
      gtsam::Point2 p2(xs[1],
                       (-C(1, 0) * 2 * xs[1] - C(2, 1) * 2) / (2 * C(1, 1)));

      // append extrema to set of points
      // 方程组解出的四个交点
      points.push_back(p0);
      points.push_back(p1);
      points.push_back(p2);
      points.push_back(p3);
    }
    catch (std::runtime_error &e)
    {
      throw e;
    }

    // 然后需要判断上面解出的四个点是不是最优的（矩形框最小）
    // 计算图像左右，上下边缘与对偶锥曲线的交点。
    // intersection of conic and line at X = 0
    try
    {
      gtsam::Vector2 ys = utils::getConicPointsAtX(C, 0.0);
      points.push_back(gtsam::Point2(0.0, ys[0]));
      points.push_back(gtsam::Point2(0.0, ys[1]));
    }
    catch (std::runtime_error &e)
    {
    }

    // intersection of conic and line at X = width
    try
    {
      gtsam::Vector2 ys = utils::getConicPointsAtX(C, imageWidth);
      points.push_back(gtsam::Point2(imageWidth, ys[0]));
      points.push_back(gtsam::Point2(imageWidth, ys[1]));
    }
    catch (std::runtime_error &e)
    {
    }

    // intersection of conic and line at Y = 0
    try
    {
      gtsam::Vector2 xs = utils::getConicPointsAtY(C, 0.0);
      points.push_back(gtsam::Point2(xs[0], 0.0));
      points.push_back(gtsam::Point2(xs[1], 0.0));
    }
    catch (std::runtime_error &e)
    {
    }

    // intersection of conic and line at Y = height
    try
    {
      gtsam::Vector2 xs = utils::getConicPointsAtY(C, imageHeight);
      points.push_back(gtsam::Point2(xs[0], imageHeight));
      points.push_back(gtsam::Point2(xs[1], imageHeight));
    }
    catch (std::runtime_error &e)
    {
    }

    // push back any captured image boundaries
    // 函数检查给定图像的四个角点(左上、右上、左下、右下)是否在当前对偶锥曲线中。
    gtsam::Point2 i1(0.0, 0.0);
    gtsam::Point2 i2(0.0, imageHeight);
    gtsam::Point2 i3(imageWidth, 0.0);
    gtsam::Point2 i4(imageWidth, imageHeight);
    if (this->contains(i1))
    {
      points.push_back(i1);
    }
    if (this->contains(i2))
    {
      points.push_back(i2);
    }
    if (this->contains(i3))
    {
      points.push_back(i3);
    }
    if (this->contains(i4))
    {
      points.push_back(i4);
    }

    // only accept non-imaginary points within image boundaries
    /// NOTE: it's important that contains includes points on the boundary
    /// ^ such that the fov intersect points count as valid
    // 包含边界上的点很重要，因为这可以让视场(FOV)与图像边界的交点被视为有效。
    // 检查上面计算出的点是否在给定的图像范围内。
    std::vector<gtsam::Point2> validPoints;
    for (auto point : points)
    {
      if (imageBounds.contains(point))
      {
        validPoints.push_back(point);
      }
    }
    // 如果找不到任何点，则会引发一个std::runtime_error异常，表示输入的圆锥曲线不可见。
    if (validPoints.size() < 1)
    {
      throw std::runtime_error(
          "no valid conic points inside image dimensions, implies quadric not "
          "visible");
      // simpleBounds.print("Failed SimpleBounds:");
      // throw QuadricProjectionException("No valid conic points inside image
      // dimensions, implies quadric not visible");
    }

    // 找到有效点后，计算出所有点中x和y坐标的最小和最大值。创建一个名为“smartBounds”的矩形框。
    auto minMaxX = std::minmax_element(
        validPoints.begin(), validPoints.end(),
        [](const gtsam::Point2 &lhs, const gtsam::Point2 &rhs)
        {
          return lhs.x() < rhs.x();
        });
    auto minMaxY = std::minmax_element(
        validPoints.begin(), validPoints.end(),
        [](const gtsam::Point2 &lhs, const gtsam::Point2 &rhs)
        {
          return lhs.y() < rhs.y();
        });
    // take the max/min of remaining points
    AlignedBox2 smartBounds(minMaxX.first->x(), minMaxY.first->y(),
                            minMaxX.second->x(), minMaxY.second->y());

    // calculate jacobians
    // 计算最小边界框相对于曲面拟合参数向量的Jacobian矩阵，该矩阵描述了最小边界框如何响应拟合曲面参数向量的变化。
    if (H)
    {
      // we want to derive wrt vector output and matrix input
      auto bounds_funptr =
          [&](const gtsam::Matrix33 &conic_matrix) -> gtsam::Vector
      {
        return DualConic(conic_matrix)
            .smartBounds(calibration, boost::none)
            .vector();
      };

      // cast to boost::function for numericalDerivative
      auto boost_funptr(
          static_cast<std::function<gtsam::Vector(const gtsam::Matrix33 &)>>(
              bounds_funptr));

      // calculate derivative of conic_matrix wrt quadric vector
      Eigen::Matrix<double, 4, 9> db_dC =
          gtsam::numericalDerivative11(boost_funptr, this->matrix(), 1e-6);

      // set jacobian to numerical derivative
      *H = db_dC;
    }
    return smartBounds;
  }

  /* ************************************************************************* */
  // 函数判断一个二次锥曲线是否退化。
  // 其实现方式为先求出二次锥曲线的对偶矩阵的逆，如果逆矩阵的行列式为0，或者逆矩阵中有元素为无穷大或NaN，则认为该二次锥曲线退化。
  bool DualConic::isDegenerate(void) const
  {
    gtsam::Matrix33 C = dC_.inverse();
    if (C.determinant() == 0.0 || C.array().isInf().any() ||
        C.array().isNaN().any())
    {
      return true;
    }
    return false;
  }

  /* ************************************************************************* */
  // 函数判断一个二次锥曲线是否为椭圆。
  // 其实现方式为先求出二次锥曲线的对偶矩阵的逆，将逆矩阵缩放为最后一个元素为1，
  // 再提取出逆矩阵左上角的2x2子矩阵，判断该2x2子矩阵的行列式是否为正数，如果是，则认为该二次锥曲线为椭圆。
  bool DualConic::isEllipse(void) const
  {
    gtsam::Matrix33 C = dC_.inverse();
    C = C / C(2, 2);
    bool isDegenerate = C.determinant() == 0.0;
    if (!isDegenerate)
    {
      gtsam::Matrix22 A33 = C.block(0, 0, 2, 2);
      return (A33.determinant() > 0);
    }
    return false;
  }

  /* ************************************************************************* */
  // 函数判断一个点是否在一个二次锥曲线上。
  // 其实现方式为先将点表示为齐次坐标形式，
  // 然后将点与二次锥曲线的矩阵的逆矩阵相乘，再将其与点向量进行内积，
  // 如果内积小于一个给定的阈值，则认为该点在二次锥曲线上。
  bool DualConic::contains(const gtsam::Point2 &p) const
  {
    gtsam::Vector3 point = (gtsam::Vector3() << p, 1.0).finished();
    double pointError = point.transpose() * this->matrix().inverse() * point;

    // apply a threshold due to noisy matrix inversion
    double thresh = 1e-10;
    if (pointError <= thresh)
    {
      return true;
    }
    return false;
  }

  /* ************************************************************************* */
  void DualConic::print(const string &s) const
  {
    cout << s << " : \n"
         << dC_ << endl;
  }

  /* ************************************************************************* */
  // 函数判断两个二次锥曲线是否相等。
  // 其实现方式为将两个二次锥曲线先进行归一化，即对其对偶矩阵除以最后一个元素的平方，
  // 然后判断其对偶矩阵是否在给定的误差范围内近似相等。
  bool DualConic::equals(const DualConic &other, double tol) const
  {
    return this->normalize().matrix().isApprox(other.normalize().matrix(), tol);
  }

} // namespace gtsam_quadrics
