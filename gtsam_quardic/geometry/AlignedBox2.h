/* ----------------------------------------------------------------------------
这段代码定义了一个名为AlignedBox2的类，用于表示一个二维平面上的矩形框，用左上右下（Top-Left-Bottom-Right，TLBR）表示法来表示。这个类包括了一些属性和方法：

属性：
  tlbr_：代表矩形框的四个坐标，用一个包含四个元素的向量（gtsam::Vector4）来表示。其中tlbr_[0] 和 tlbr_[1] 是矩形框左上角的坐标，tlbr_[2] 和 tlbr_[3] 是矩形框右下角的坐标。

方法：
  属性：构造函数：有三种构造函数，分别是默认构造函数、从 double 类型的 xmin、ymin、xmax、ymax 构造和从 Vector4 类型的 tlbr 构造。

  各种访问方法：比如 xmin、ymin、xmax、ymax、vector、minPoint、maxPoint、center、width 和 height 等，用于获取矩形框的各种属性信息。

  交并运算方法：contains、intersects、iou 等，用于判断一个矩形框是否包含另一个矩形框、是否相交、计算它们之间的交并比等。

  线段方法：lines，用于返回矩形框的各条线段方程。

  测试组方法：print 和 equals，用于打印测试信息和比较两个矩形框是否相等。

此外，这个类还包含了一个 Vector3Vector 类型的 typedef 定义，用于 Python 封装。

最后，使用了 gtsam::traits 模板，将该类添加到测试组中，以支持 GTSAM 库的测试机制。

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.h
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 2D bounding box
 */

#pragma once

#include <gtsam/base/Testable.h>
#include <gtsam/geometry/Pose3.h>

#include <vector>

namespace gtsam_quadrics
{

  /**
   * @class AlignedBox2
   * An axis aligned 2D bounding box (xmin,ymin,xmax,ymax)
   */
  class AlignedBox2
  {
  protected:
    gtsam::Vector4 tlbr_; ///< xmin,ymin,xmax,ymax
    // 代表一个二维平面上的矩形框，用一个包含四个元素的向量（gtsam::Vector4）来表示。
    // 其中 tlbr_[0] 和 tlbr_[1] 是矩形框左上角的坐标，tlbr_[2] 和 tlbr_[3] 是矩形框右下角的坐标。
    // 这种表示方法也被称为“左上右下（Top-Left-Bottom-Right，TLBR）”表示法

  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    // 用于为类提供自定义的内存分配函数，以确保它们的内存对齐和提高内存读取性能。
    // 在使用 Eigen 库的时候，如果我们需要在类中使用 Eigen 的矩阵或向量等类型，
    // 那么我们就需要使用该宏定义来重载 new 和 delete 操作符，以确保内存对齐。
    // 当一个类使用 EIGEN_MAKE_ALIGNED_OPERATOR_NEW 宏定义后，该类在内存分配时会自动进行对齐。
    // 使用这个宏定义可以确保类的实例的内存地址都是 16 字节对齐，这对于许多 SSE 和 AVX 操作是必要的。

    /// @name Constructors and named constructors
    /// @{

    /** Default constructor */

    AlignedBox2() : tlbr_(0, 0, 0, 0){};

    /** Constructor from doubles */
    AlignedBox2(const double &xmin, const double &ymin, const double &xmax,
                const double &ymax)
        : tlbr_(xmin, ymin, xmax, ymax){};

    /**
     * Constructor from vector
     * @param tlbr vector of xmin,ymin,xmax,ymax (Vector4)
     */
    AlignedBox2(const gtsam::Vector4 &tlbr) : tlbr_(tlbr){};

    /// @}
    /// @name Class accessors
    /// @{

    /** Get xmin */
    double xmin() const { return tlbr_[0]; }

    /** Get ymin */
    double ymin() const { return tlbr_[1]; }

    /** Get xmax */
    double xmax() const { return tlbr_[2]; }

    /** Get ymax */
    double ymax() const { return tlbr_[3]; }

    /** Returns box in xmin,ymin,xmax,ymax vector */
    gtsam::Vector4 vector() const { return tlbr_; };

    /** Returns Point2(xmin, ymin) */
    gtsam::Point2 minPoint() const { return gtsam::Vector2(xmin(), ymin()); }

    /** Returns Point2(xmax, ymax) */
    gtsam::Point2 maxPoint() const { return gtsam::Vector2(xmax(), ymax()); }

    /** Returns box centre */
    gtsam::Point2 center() const
    {
      return gtsam::Vector2((xmin() + xmax()) / 2, (ymin() + ymax()) / 2);
    }

    /** Returns box width */
    double width() const { return std::abs(xmax() - xmin()); }

    /** Returns box height */
    double height() const { return std::abs(ymax() - ymin()); }

    /// @}
    /// @name Class methods
    /// @{

    /** Returns equation of boxes lines */
    std::vector<gtsam::Vector3> lines() const;

    /**
     * Returns true if this contains the point
     * Points intersecting a line are considered containing
     */
    bool contains(const gtsam::Point2 &point) const;

    /**
     * Returns true if this completely contains other box
     * Edges touching are considered contained
     */
    bool contains(const AlignedBox2 &other) const;

    /**
     * Returns true if this intersects other box
     * Edges touching are considered not intersecting
     * NOTE: assumes xmin < xmax, ymin < ymax
     */
    bool intersects(const AlignedBox2 &other) const;

    /**
     * Calculates the standard intersection over union
     * between two axis aligned bounding boxes.
     */
    double iou(const AlignedBox2 &other) const;

    /// @}
    /// @name Testable group traits
    /// @{

    /** Prints the box vector with optional string */
    void print(const std::string &s = "") const;

    /** Compares two boxes */
    bool equals(const AlignedBox2 &other, double tol = 1e-9) const;

    /// @}
  };

  // Add vector<> typedef for python wrapper
  typedef std::vector<gtsam::Vector3> Vector3Vector;

} // namespace gtsam_quadrics

/** \cond PRIVATE */
// Add to testable group
// 这段代码定义了一个特化的gtsam::traits模板，用于指定gtsam_quadrics::AlignedBox2类型的特性。
// 在这里，AlignedBox2类型被指定为Testable类型，这意味着它具有可测试的特性，可以使用Testable中定义的方法进行测试。
// 通过这个特化的gtsam::traits模板，可以对AlignedBox2类型进行统一的处理，比如实现序列化、反序列化等操作，可以方便的集成到GTSAM库的使用中。
template <>
struct gtsam::traits<gtsam_quadrics::AlignedBox2>
    : public gtsam::Testable<gtsam_quadrics::AlignedBox2>
{
};
/** \endcond */
