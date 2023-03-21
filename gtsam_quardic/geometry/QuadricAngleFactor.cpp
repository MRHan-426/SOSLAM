/* ----------------------------------------------------------------------------

这段代码定义了三个函数：
QuadricAngleFactor::evaluateError @ 实测角度与 quadric 角度之间的误差 @
QuadricAngleFactor::print
QuadricAngleFactor::equals
这些函数都是属于名为 QuadricAngleFactor 的类的成员函数。

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricAngleFactor.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/geometry/QuadricAngleFactor.h>

#include <boost/bind/bind.hpp>

#define NUMERICAL_DERIVATIVE false

using namespace std;

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 函数的输入是 ConstrainedDualQuadric 对象和可选的雅可比矩阵 H，
  // 该函数首先通过 quadric.pose().rotation() 获取给定 quadric 的旋转部分，
  // 然后使用该旋转部分计算出实测角度与 quadric 角度之间的误差，并将误差存储在 error 变量中。
  // 如果输入参数 H 不为空，该函数还会计算误差函数相对于 quadric 的导数，将结果存储在 H 变量中。
  gtsam::Vector QuadricAngleFactor::evaluateError(
      const ConstrainedDualQuadric &quadric,
      boost::optional<gtsam::Matrix &> H) const
  {
    // evaluate error
    // 该函数首先通过 quadric.pose().rotation() 获取给定 quadric 的旋转部分，
    gtsam::Rot3 QRot = quadric.pose().rotation();

    // 然后使用该旋转部分计算出实测角度与 quadric 角度之间的误差，并将误差存储在 error 变量中。
    gtsam::Vector3 error = measured_.localCoordinates(QRot);
    // Rot3::LocalCoordinates(quadric.pose().rotation());

    // funPtr 被用于计算误差函数相对于 quadric 的导数。
    // funPtr 的实际函数指针是通过 boost::bind 函数绑定到 QuadricAngleFactor::evaluateError 函数上的，
    // 这个函数指针包装对象可以在需要时通过调用 funPtr(quadric) 来执行 QuadricAngleFactor::evaluateError 函数。
    std::function<gtsam::Vector(const ConstrainedDualQuadric &)> funPtr(
        boost::bind(&QuadricAngleFactor::evaluateError, this,
                    boost::placeholders::_1, boost::none));

    // 如果输入参数 H 不为空，该函数还会计算误差函数相对于 quadric 的导数，将结果存储在 H 变量中。
    if (H)
    {
      Eigen::Matrix<double, 3, 9> de_dr =
          gtsam::numericalDerivative11(funPtr, quadric, 1e-6);
      *H = de_dr;
    }
    return error;
  }

  /* ************************************************************************* */
  // 函数的输入是一个字符串 s 和一个 gtsam::KeyFormatter 对象 keyFormatter，
  // 该函数用于打印一个描述该因子的字符串，其中包括该因子的关键字和噪声模型。
  void QuadricAngleFactor::print(const std::string &s,
                                 const gtsam::KeyFormatter &keyFormatter) const
  {
    cout << s << "QuadricAngleFactor(" << keyFormatter(key()) << ")" << endl;
    cout << "    NoiseModel: ";
    noiseModel()->print();
    cout << endl;
  }

  /* ************************************************************************* */
  // 函数的输入是一个 QuadricAngleFactor 对象 other 和一个 double 类型的容差 tol，该函数返回一个布尔值。
  // 该函数比较两个 QuadricAngleFactor 对象是否相等，其中噪声模型相等且关键字相同的因子被认为是相等的。
  bool QuadricAngleFactor::equals(const QuadricAngleFactor &other,
                                  double tol) const
  {
    bool equal =
        noiseModel()->equals(*other.noiseModel(), tol) && key() == other.key();
    return equal;
  }

} // namespace gtsam_quadrics
