/* ----------------------------------------------------------------------------

QuadricCamera的类，该类表示一种能够投影二次曲面的相机。
其中定义了三个公共成员函数：
    transformToImage()、project()和project()。两个投影函数。
    1.transformToImage()是一个静态函数，用于计算一个姿态和内参数为给定参数的投影矩阵。
    2.project()函数用于将一个给定的三维二次曲面投影到存储的相机姿态和内参数所描述的二维图像平面上，并返回一个投影后的双圆锥体。
    3.最后一个project()函数用于将一个给定的二维矩形框投影到三维空间中的平面上，并返回表示该平面的四维向量。

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricCamera.h
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a class responsible for projecting quadrics
 */

#pragma once

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/base/types.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <gtsam/geometry/PinholePose.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/DualConic.h>

namespace gtsam_quadrics {

/**
 * @class QuadricCamera
 * A camera that projects quadrics
 */
class QuadricCamera {
 public:
  /** Static projection matrix */
  static gtsam::Matrix34 transformToImage(
      const gtsam::Pose3& pose,
      const boost::shared_ptr<gtsam::Cal3_S2>& calibration);

  /**
   * Project a quadric at the stored 3D pose and calibration
   * @param quadric the 3D quadric surface to be projected
   * @return the projected dual conic
   */
  static DualConic project(const ConstrainedDualQuadric& quadric,
                           const gtsam::Pose3& pose,
                           const boost::shared_ptr<gtsam::Cal3_S2>& calibration,
                           gtsam::OptionalJacobian<9, 9> dC_dq = boost::none,
                           gtsam::OptionalJacobian<9, 6> dC_dx = boost::none);

  /** Project box to planes */
  static std::vector<gtsam::Vector4> project(
      const AlignedBox2& box, const gtsam::Pose3& pose,
      const boost::shared_ptr<gtsam::Cal3_S2>& calibration);
};

}  // namespace gtsam_quadrics
