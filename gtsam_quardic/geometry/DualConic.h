/* ----------------------------------------------------------------------------
这段代码中的参数定义了一个名为DualConic的类，该类表示二次曲线。以下是该类中的一些重要成员函数：

DualConic()：默认构造函数，创建一个以原点为中心的单位圆。
DualConic(const gtsam::Matrix33& dC)：构造函数，从3x3矩阵创建一个二次曲线。
DualConic(const gtsam::Pose2& pose, const gtsam::Vector2& radii)：构造函数，从2D位姿和轴长度创建一个椭圆。

matrix()：返回3x3二次方程矩阵。

normalize()：返回归一化的对偶锥曲线。

bounds(gtsam::OptionalJacobian<4, 9> H = boost::none)：返回在图像平面上的标准2D边界，不考虑图像尺寸。

smartBounds(const boost::shared_ptr<gtsam::Cal3_S2>& calibration, gtsam::OptionalJacobian<4, 9> H = boost::none)：返回一个物体检测器将看到的边界，仔细处理与图像边界的相交。注意：假设圆锥曲线是椭圆形和非退化的（将引发std::runtime_error）失败：如果无法看到二次曲面。

isDegenerate()：返回真如果圆锥曲线是退化的。

isEllipse()：返回真如果圆锥曲线是椭圆形或圆形。内部计算退化。

contains(const gtsam::Point2& p)：返回真如果对偶锥曲线包含点。曲线边缘上的点被认为是包含的。

print(const std::string& s = "")：打印带有可选字符串的对偶锥曲线。

equals(const DualConic& other, double tol = 1e-9)：比较两个对偶锥曲线，考虑归一化。

 * -------------------------------------------------------------------------- */

/**
 * @file DualConic.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a dual conic
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>

namespace gtsam_quadrics
{

  /**
   * @class DualConic
   * A dual conic (Matrix33)
   */
  class DualConic
  {
  protected:
    //  在代码中，dC_是DualConic类中的一个私有成员变量，它是一个3x3的矩阵，表示二次曲线方程的系数矩阵。
    //  这个矩阵被称为“对偶锥曲线矩阵”，对应着二维几何中的一个对偶锥曲线（例如椭圆、双曲线等），
    //  可以通过它来表示一个对偶锥曲线的性质，如其形状、位置等。
    //  DualConic类中的很多方法都是围绕着这个矩阵展开的，例如获取矩阵、归一化、计算边界、判断是否是椭圆等。
    gtsam::Matrix33 dC_; ///< 3x3 matrix of the quadratic equation

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor: unit circle at origin */
    DualConic();

    /** Constructor from 3x3 matrix */
    DualConic(const gtsam::Matrix33 &dC) : dC_(dC){};

    /** Create ellipse from 2D pose and axis lengths */
    DualConic(const gtsam::Pose2 &pose, const gtsam::Vector2 &radii);

    /// @}
    /// @name Class methods
    /// @{

    /** Return 3x3 conic matrix */
    gtsam::Matrix33 matrix(void) const { return dC_; }

    /** Return normalized dual conic */
    DualConic normalize(void) const;

    /**
     * Returns the standard 2D bounds on the image plane
     * with no consideration for image dimensions
     */
    AlignedBox2 bounds(gtsam::OptionalJacobian<4, 9> H = boost::none) const;

    /**
     * Returns the bounds as an object detector would see
     * Carefully handling the intersection with the image boundaries
     * NOTE: assumes conic is elliptical and non-degenerate (will throw
     * std::runtimeerror) FAILS: if quadric is not visible
     */
    AlignedBox2 smartBounds(const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                            gtsam::OptionalJacobian<4, 9> H = boost::none) const;

    /**
     * Returns true if conic section is degenerate
     * Using det(C) as opposed to sign(eigenvalues)
     */
    bool isDegenerate(void) const;

    /**
     * Returns true if conic section is elliptical or circular
     * Internally calculates degeneracy
     */
    bool isEllipse(void) const;

    /**
     * Returns true if dual conic contains the point
     * Points on the edge of the conic are considered contained
     */
    bool contains(const gtsam::Point2 &p) const;

    /// @}
    /// @name Testable group traits
    /// @{

    /** Prints the dual conic with optional string */
    void print(const std::string &s = "") const;

    /** Compares two dual conics accounting for normalization */
    bool equals(const DualConic &other, double tol = 1e-9) const;

    /// @}
  };

} // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add DualConic to Testable group
template <>
struct gtsam::traits<gtsam_quadrics::DualConic>
    : public gtsam::Testable<gtsam_quadrics::DualConic>
{
};
/** \endcond */
