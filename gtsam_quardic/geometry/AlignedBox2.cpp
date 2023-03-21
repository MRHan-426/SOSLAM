/* ----------------------------------------------------------------------------

这是一个名为 AlignedBox2 的类，实现了一些基于二维轴对齐矩形框（axis-aligned bounding box，AABB）的操作。
该类定义在命名空间 gtsam_quadrics 中。

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 2D bounding box
 */

#include <gtsam_quadrics/geometry/AlignedBox2.h>

using namespace std;

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 返回该矩形框的四条边所在的直线的单位法向量，即对应的四个三维向量，每个向量的 $z$ 分量为对应边的偏移（见下面示例代码）。
  // 这个函数在计算二维轴对齐矩形框的凸包时有用。
  std::vector<gtsam::Vector3> AlignedBox2::lines() const
  {
    std::vector<gtsam::Vector3> mLines;
    mLines.push_back(gtsam::Vector3(1, 0, -tlbr_[0]));
    mLines.push_back(gtsam::Vector3(0, 1, -tlbr_[1]));
    mLines.push_back(gtsam::Vector3(1, 0, -tlbr_[2]));
    mLines.push_back(gtsam::Vector3(0, 1, -tlbr_[3]));
    return mLines;
  }

  /* ************************************************************************* */
  // 给定一个二维点，判断该点是否在矩形框内部。返回 true 或 false。
  bool AlignedBox2::contains(const gtsam::Point2 &point) const
  {
    if (point.x() >= xmin() && point.x() <= xmax() && point.y() >= ymin() &&
        point.y() <= ymax())
    {
      return true;
    }
    return false;
  }

  /* ************************************************************************* */
  // 给定另一个二维轴对齐矩形框 other，判断该矩形框是否被当前矩形框完全包含。返回 true 或 false。
  bool AlignedBox2::contains(const AlignedBox2 &other) const
  {
    return (other.xmin() >= this->xmin() && other.xmax() <= this->xmax() &&
            other.ymin() >= this->ymin() && other.ymax() <= this->ymax());
  }

  /* ************************************************************************* */
  // 给定另一个二维轴对齐矩形框 other，判断该矩形框是否和当前矩形框有重叠。返回 true 或 false。
  bool AlignedBox2::intersects(const AlignedBox2 &other) const
  {
    if (this->contains(other) || other.contains(*this))
    {
      return false;
    }
    return !(this->xmin() > other.xmax() || this->xmax() < other.xmin() ||
             this->ymin() > other.ymax() || this->ymax() < other.ymin());
  }

  /* ************************************************************************* */
  // 计算当前矩形框和另一个矩形框 other 的 IoU（intersection over union）。返回 IoU 值，范围在 $[0, 1]$ 之间。
  double AlignedBox2::iou(const AlignedBox2 &other) const
  {
    AlignedBox2 inter_box(std::max(this->xmin(), other.xmin()),
                          std::max(this->ymin(), other.ymin()),
                          std::min(this->xmax(), other.xmax()),
                          std::min(this->ymax(), other.ymax()));

    if ((inter_box.xmax() < inter_box.xmin()) ||
        (inter_box.ymax() < inter_box.ymin()))
    {
      return 0.0;
    }

    double inter_area = inter_box.width() * inter_box.height();
    double this_area = this->width() * this->height();
    double other_area = other.width() * other.height();

    double iou = inter_area / (this_area + other_area - inter_area);
    assert(iou >= 0.0);
    assert(iou <= 1.0);
    return iou;
  }

  /* ************************************************************************* */
  // 打印当前矩形框的左上角和右下角坐标，用一个字符串 s 作为前缀。
  void AlignedBox2::print(const std::string &s) const
  {
    cout << s << this->vector().transpose() << endl;
  }

  /* ************************************************************************* */
  // 给定另一个二维轴对齐矩形框 other 和一个公差 tol，判断当前矩形框是否和 other 相等。
  // 如果两个矩形框的四个坐标分量差的绝对值都小于 tol，则认为它们相等。
  bool AlignedBox2::equals(const AlignedBox2 &other, double tol) const
  {
    return tlbr_.isApprox(other.tlbr_, tol);
  }

} // namespace gtsam_quadrics
