/* ----------------------------------------------------------------------------

这段代码定义了一个命名空间 gtsam_quadrics，其中包含一个名为 BoundingBoxFactor 的类。

这个类是一个测量模型因子，它将 位姿(Pose3) 和 ConstrainedDualQuadric 对象的 AlignedBox3 限制联系在一起。

这个类有三个构造函数，它们允许创建一个测量模型因子对象，并传递所需的参数：
  测量值、相机校准、位姿关键字、对象/地标关键字、噪声模型和测量模型类型。

此外，这个类还提供了访问器函数，允许用户获取测量值、位姿关键字和对象/地标关键字。

BoundingBoxFactor 类中还定义了一些方法，例如:
  evaluateError 函数用于计算给定相机位姿和受限双重二次曲面的误差。
  evaluateH1 和 evaluateH2 函数分别计算相机位姿和受限双重二次曲面的误差导数。

此外，BoundingBoxFactor 类还包含了一个枚举 MeasurementModel，用于声明使用哪个误差函数。

最后，这个文件还定义了一个测试组，用于测试 BoundingBoxFactor 类的相等性。

 * -------------------------------------------------------------------------- */

/**
 * @file BoundingBoxFactor.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

#pragma once

#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/Expression.h>
#include <gtsam/nonlinear/NonlinearFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>

namespace gtsam_quadrics
{

  /**
   * @class BoundingBoxFactor
   * AlignedBox3 factor between Pose3 and ConstrainedDualQuadric
   * Projects the quadric at the current pose estimates,
   * Calculates the bounds of the dual conic,
   * and compares this to the measured bounding box.
   */
  class BoundingBoxFactor
      : public gtsam::NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric>
  {
  public:
    enum MeasurementModel
    {
      STANDARD,
      TRUNCATED
    }; ///< enum to declare which error function to use

  protected:
    AlignedBox2 measured_;                          ///< measured bounding box
    boost::shared_ptr<gtsam::Cal3_S2> calibration_; ///< camera calibration

    // 这段代码定义了一个类型为NoiseModelFactor2的别名。
    // NoiseModelFactor2是GTSAM中的一个因子类型，用于定义一个非线性约束方程，
    // 它将一个Pose3变量和一个ConstrainedDualQuadric变量联系起来，同时给定一个噪声模型。
    // 相机或激光雷达的观测通常可以通过一个Pose3变量来描述，而地图中的物体可以用ConstrainedDualQuadric来表示。
    typedef NoiseModelFactor2<gtsam::Pose3, ConstrainedDualQuadric>
        Base; ///< base class has keys and noisemodel as private members
    // Base是一个基类类型，用于实现具体的约束方程。
    // 在这个别名定义中，将Base的模板参数设置为gtsam::Pose3和ConstrainedDualQuadric，
    // 这样就可以通过继承和实现Base的纯虚函数来定义具体的约束方程。

    MeasurementModel measurementModel_;

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    BoundingBoxFactor()
        : measured_(0., 0., 0., 0.), measurementModel_(STANDARD){};

    /** Constructor from measured box, calbration, dimensions and posekey,
     * quadrickey, noisemodel */
    // 测量值、相机校准、位姿关键字、对象/地标关键字、噪声模型和测量模型类型。
    BoundingBoxFactor(const AlignedBox2 &measured,
                      const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                      const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                      const gtsam::SharedNoiseModel &model,
                      const MeasurementModel &errorType = STANDARD)
        : Base(model, poseKey, quadricKey), // 这里的冒号 : 表示这是一个构造函数的初始化列表，用于初始化类成员变量
          measured_(measured),
          calibration_(calibration),
          measurementModel_(errorType){};

    /** Constructor from measured box, calbration, dimensions and posekey,
     * quadrickey, noisemodel */
    BoundingBoxFactor(const AlignedBox2 &measured,
                      const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                      const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                      const gtsam::SharedNoiseModel &model,
                      const std::string &errorString)
        : Base(model, poseKey, quadricKey),
          measured_(measured),
          calibration_(calibration)
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
            "\" is not a valid option for initializing a BoundingBoxFactor");
      }
    }

    /// @}
    /// @name Class accessors
    /// @{

    /** Returns the measured bounding box */
    AlignedBox2 measurement() const { return AlignedBox2(measured_.vector()); }

    // poseKey和quadricKey分别指代位姿（Pose）和对象（Object）的Key
    // 在这个因子模型中，这两个变量的Key被用来描述物体在相机坐标系下的位姿和轮廓，
    // 从而将这个因子模型与其他因子模型（如视觉因子模型）结合起来，实现多传感器融合的目的。
    gtsam::Key poseKey() const { return key1(); } // Returns the pose key

    gtsam::Key objectKey() const { return key2(); } // Returns the object/landmark key

    /// @}
    /// @name Class methods
    /// @{

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

/** \cond PRIVATE */
// Add to testable group
template <>
struct gtsam::traits<gtsam_quadrics::BoundingBoxFactor>
    : public gtsam::Testable<gtsam_quadrics::BoundingBoxFactor>
{
};
/** \endcond */
