/* ----------------------------------------------------------------------------

这段代码定义了一个名为AlignedBox3的类，该类包含以下几个成员函数：

AlignedBox3::AlignedBox3(const double& xmin, const double& xmax, const double& ymin, const double& ymax, const double& zmin, const double& zmax)
该函数是AlignedBox3类的构造函数，用于创建一个三维空间中的轴对齐盒子。输入参数xmin, xmax, ymin, ymax, zmin和zmax分别代表盒子在x、y和z三个方向的最小坐标和最大坐标。

gtsam::Vector3 AlignedBox3::dimensions() const
该函数用于计算盒子的三个维度（宽度、高度和深度）并返回一个包含这些维度信息的gtsam::Vector3向量。

gtsam::Vector3 AlignedBox3::centroid() const
该函数用于计算盒子的中心点并返回一个包含这个点坐标的gtsam::Vector3向量。

double AlignedBox3::iou(const AlignedBox3& other) const
该函数用于计算两个盒子之间的交并比（Intersection over Union，IOU），其中输入参数other代表另外一个盒子对象。该函数首先计算两个盒子的交集，并判断是否存在交集。如果两个盒子没有交集，则返回0。如果两个盒子有交集，则计算两个盒子的交集体积和并集体积，并返回它们的比值作为IOU。

void AlignedBox3::print(const std::string& s) const
该函数用于将盒子对象的数据打印到标准输出流（stdout）中，其中输入参数s为一个字符串，用于表示该输出内容的前缀信息。

bool AlignedBox3::equals(const AlignedBox3& other, double tol) const
该函数用于判断两个盒子对象是否相等。其中输入参数other代表另外一个盒子对象，tol为一个容差参数，用于判断两个盒子对象的坐标是否相等。如果两个盒子对象的坐标相差小于容差tol，则返回true；否则返回false。

 * -------------------------------------------------------------------------- */

/**
 * @file AlignedBox2.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief an axis aligned 3D bounding box
 */

#include <gtsam_quadrics/geometry/AlignedBox3.h>

using namespace std;

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 该函数是AlignedBox3类的构造函数，用于创建一个三维空间中的轴对齐盒子。
  // 输入参数xmin, xmax, ymin, ymax, zmin和zmax分别代表盒子在x、y和z三个方向的最小坐标和最大坐标。
  AlignedBox3::AlignedBox3(const double &xmin, const double &xmax,
                           const double &ymin, const double &ymax,
                           const double &zmin, const double &zmax)
  {
    xxyyzz_ = (gtsam::Vector6() << xmin, xmax, ymin, ymax, zmin, zmax).finished();
  }

  /* ************************************************************************* */
  // 该函数用于计算盒子的三个维度（宽度、高度和深度）并返回一个包含这些维度信息的gtsam::Vector3向量。
  gtsam::Vector3 AlignedBox3::dimensions() const
  {
    return (gtsam::Vector3() << xmax() - xmin(), ymax() - ymin(), zmax() - zmin())
        .finished();
  }

  /* ************************************************************************* */
  // 该函数用于计算盒子的中心点并返回一个包含这个点坐标的gtsam::Vector3向量。
  gtsam::Vector3 AlignedBox3::centroid() const
  {
    return (gtsam::Vector3() << xmin() + xmax(), ymin() + ymax(), zmin() + zmax())
               .finished() /
           2.0;
  }

  /* ************************************************************************* */
  // 该函数用于计算两个盒子之间的交并比（Intersection over Union，IOU），
  // 其中输入参数other代表另外一个盒子对象。该函数首先计算两个盒子的交集，并判断是否存在交集。
  // 如果两个盒子没有交集，则返回0。
  // 如果两个盒子有交集，则计算两个盒子的交集体积和并集体积，并返回它们的比值作为IOU。
  double AlignedBox3::iou(const AlignedBox3 &other) const
  {
    AlignedBox3 inter_box(std::max(this->xmin(), other.xmin()),
                          std::min(this->xmax(), other.xmax()),
                          std::max(this->ymin(), other.ymin()),
                          std::min(this->ymax(), other.ymax()),
                          std::max(this->zmin(), other.zmin()),
                          std::min(this->zmax(), other.zmax()));

    if ((inter_box.xmax() < inter_box.xmin()) ||
        (inter_box.ymax() < inter_box.ymin()) ||
        (inter_box.zmax() < inter_box.zmin()))
    {
      return 0.0;
    }

    double inter_volume = inter_box.volume();
    double iou = inter_volume / (this->volume() + other.volume() - inter_volume);

    assert(iou >= 0.0);
    assert(iou <= 1.0);
    return iou;
  }

  /* ************************************************************************* */
  // 该函数用于打印盒子xyz方向的最小值和最大值，
  // 其中输入参数s为一个字符串，用于表示该输出内容的前缀信息。
  void AlignedBox3::print(const std::string &s) const
  {
    cout << s << this->vector().transpose() << endl;
  }

  /* ************************************************************************* */
  // 该函数用于判断两个盒子对象是否相等。
  // 其中输入参数other代表另外一个盒子对象，tol为一个容差参数，用于判断两个盒子对象的坐标是否相等。
  // 如果两个盒子对象的坐标相差小于容差tol，则返回true；否则返回false。
  bool AlignedBox3::equals(const AlignedBox3 &other, double tol) const
  {
    return xxyyzz_.isApprox(other.xxyyzz_, tol);
  }

} // namespace gtsam_quadrics
