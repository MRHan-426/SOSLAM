/* ----------------------------------------------------------------------------
这段代码定义了一个名为AlignedBox3的类，属于gtsam_quadrics命名空间。
该类代表一个三维轴对齐盒子，其中的数据成员是一个6维向量，存储了盒子的最小和最大的xyz坐标值。类中定义了一些成员函数：

AlignedBox3::AlignedBox3(const double& xmin, const double& xmax, const double& ymin, const double& ymax, const double& zmin, const double& zmax)：构造函数，初始化AlignedBox3对象的最小和最大xyz坐标值；
gtsam::Vector3 AlignedBox3::dimensions() const：计算并返回盒子的长、宽、高，以3维向量形式返回；
gtsam::Vector3 AlignedBox3::centroid() const：计算并返回盒子的中心点坐标，以3维向量形式返回；
double AlignedBox3::iou(const AlignedBox3& other) const：计算并返回两个盒子的交并比(IOU)，其中参数other是另一个AlignedBox3对象；
void AlignedBox3::print(const std::string& s) const：将AlignedBox3对象的最小和最大xyz坐标值输出到标准输出流中，带有指定的字符串前缀s；
bool AlignedBox3::equals(const AlignedBox3& other, double tol) const：比较两个AlignedBox3对象是否相等，其中参数tol是容差。

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox3.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 3D bounding box
 */

#pragma once

#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gtsam_quadrics
{

  /**
   * @class AlignedBox3
   * An axis aligned 3D bounding box
   * (xmin, xmax, ymin, ymax, zmin, zmax)
   */
  class AlignedBox3
  {
  protected:
    gtsam::Vector6 xxyyzz_; ///< bounds vector
    // 用于存储 AlignedBox3 对象的六个坐标值（xmin、xmax、ymin、ymax、zmin、zmax）

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */
    AlignedBox3() { xxyyzz_.setZero(); };

    /** Constructor from doubles */
    AlignedBox3(const double &xmin, const double &xmax, const double &ymin,
                const double &ymax, const double &zmin, const double &zmax);

    /** Constructor from vector */
    AlignedBox3(const gtsam::Vector6 &xxyyzz) : xxyyzz_(xxyyzz){};

    /// @}
    /// @name Class accessors
    /// @{

    /** Get xmin */
    double xmin() const { return xxyyzz_[0]; }

    /** Get xmax */
    double xmax() const { return xxyyzz_[1]; }

    /** Get ymin */
    double ymin() const { return xxyyzz_[2]; }

    /** Get ymax */
    double ymax() const { return xxyyzz_[3]; }

    /** Get zmin */
    double zmin() const { return xxyyzz_[4]; }

    /** Get zmax */
    double zmax() const { return xxyyzz_[5]; }

    /** Returns box in xxyyzz vector */
    gtsam::Vector6 vector() const { return xxyyzz_; };

    /// @}
    /// @name Class methods
    /// @{

    /** Returns x,y,z lengths as a vector */
    gtsam::Vector3 dimensions() const;

    /** Returns box centroid as x,y,z vector */
    gtsam::Vector3 centroid() const;

    /** calculates volume, assuming ordered correctly */
    double volume() const
    {
      return (xmax() - xmin()) * (ymax() - ymin()) * (zmax() - zmin());
    }

    /**
     * Calculates the standard intersection over union
     * between two axis aligned bounding boxes.
     */
    double iou(const AlignedBox3 &other) const;

    /// @}
    /// @name Testable group traits
    /// @{

    /** Prints the box vector with optional string */
    void print(const std::string &s = "") const;

    /** Compares two boxes */
    bool equals(const AlignedBox3 &other, double tol = 1e-9) const;

    /// @}
  };

} // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add to testable group
template <>
struct gtsam::traits<gtsam_quadrics::AlignedBox3>
    : public gtsam::Testable<gtsam_quadrics::AlignedBox3>
{
};
/** \endcond */
