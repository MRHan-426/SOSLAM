/* ----------------------------------------------------------------------------
@ 强制将二次曲线约束成椭球形 @

这段代码实现了ConstrainedDualQuadric类的一些函数，该类表示受约束的对偶二次曲面。主要实现了以下几个函数：

ConstrainedDualQuadric::ConstrainedDualQuadric()：
构造函数，将对偶二次曲面的位姿设为gtsam::Pose3()，将其轴长设为gtsam::Vector3(1,1,1)。

ConstrainedDualQuadric::ConstrainedDualQuadric(const gtsam::Matrix44& dQ)：
构造函数，使用对偶二次曲面矩阵构造ConstrainedDualQuadric类对象。

ConstrainedDualQuadric ConstrainedDualQuadric::constrain(const gtsam::Matrix4& dual_quadric)：
将无约束的对偶二次曲面矩阵变换成受约束的对偶二次曲面，即将其对应到某个射影矩阵。

gtsam::Matrix44 ConstrainedDualQuadric::matrix(gtsam::OptionalJacobian<16, 9> dQ_dq) const：
返回该对偶二次曲面的矩阵表示。如果计算dQ_dq，则返回dQ_dq，其中16x9的Jacobian矩阵是Q关于在Pose3和轴长上的导数。

gtsam::Matrix44 ConstrainedDualQuadric::normalizedMatrix(void) const：
返回该对偶二次曲面的归一化矩阵表示。

AlignedBox3 ConstrainedDualQuadric::bounds() const：
返回该对偶二次曲面的最小矩形边界框。
 * -------------------------------------------------------------------------- */

/**
 * @file ConstrainedDualQuadric.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric
 */

#include <gtsam_quadrics/base/Utilities.h>
#include <gtsam_quadrics/geometry/AlignedBox2.h>
#include <gtsam_quadrics/geometry/ConstrainedDualQuadric.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

#include <Eigen/Eigenvalues>
#include <iostream>

using namespace std;

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 构造函数，将对偶二次曲面的位姿设为gtsam::Pose3()，将其轴长设为gtsam::Vector3(1,1,1)。
  ConstrainedDualQuadric::ConstrainedDualQuadric()
  {
    pose_ = gtsam::Pose3();
    radii_ = gtsam::Vector3(1, 1, 1);
  }

  /* ************************************************************************* */
  // 使用一个4x4的矩阵dQ来创建一个受限对偶二次曲面(Constrained Dual Quadric)对象
  ConstrainedDualQuadric::ConstrainedDualQuadric(const gtsam::Matrix44 &dQ)
  {
    *this = ConstrainedDualQuadric::constrain(dQ);
  }

  /* ************************************************************************* */
  // @ 核心函数 @
  // 这个函数的作用是将一个用于表示对偶二次曲面的4x4矩阵转换为受约束的对偶二次曲面对象
  ConstrainedDualQuadric ConstrainedDualQuadric::constrain(
      const gtsam::Matrix4 &dual_quadric)
  {
    // normalize if required
    // 函数首先检查输入矩阵的最后一个元素是否为1.0，如果不是，则将整个矩阵除以该元素以进行归一化。
    gtsam::Matrix4 normalized_dual_quadric(dual_quadric);
    if (dual_quadric(3, 3) != 1.0)
    {
      normalized_dual_quadric = dual_quadric / dual_quadric(3, 3);
    }

    // extract translation
    // 然后，从归一化的对偶二次曲面矩阵中提取平移向量。
    // translation 表示对偶二次曲面的平移向量，它是由输入的4x4矩阵中的最后一列提取而来。
    gtsam::Point3 translation(normalized_dual_quadric.block(0, 3, 3, 1));

    // calculate the point quadric matrix
    // 计算点二次曲面(point_quadric)矩阵，点二次曲面矩阵也进行了归一化。
    gtsam::Matrix4 point_quadric = normalized_dual_quadric.inverse();
    gtsam::Matrix4 normalized_point_quadric = point_quadric;
    if (point_quadric(3, 3) != 1.0)
    {
      normalized_point_quadric = point_quadric / point_quadric(3, 3);
    }

    // extract shape
    // 从点二次曲面矩阵中提取双曲面的形状（即长、宽、高），这是通过对其特征值进行计算得出的。
    // @ 论文中的计算式(10) @
    auto lambdaa = normalized_point_quadric.block(0, 0, 3, 3).eigenvalues();
    gtsam::Vector3 shape =
        Eigen::sqrt(-1.0 * normalized_point_quadric.determinant() /
                    normalized_point_quadric.block(0, 0, 3, 3).determinant() *
                    1.0 / lambdaa.array())
            .abs();

    // extract rotation
    // 函数提取旋转矩阵，这是通过将点二次曲面矩阵作为实数域上的矩阵进行特征值分解获得的。
    Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> s(
        normalized_point_quadric.block(0, 0, 3, 3));
    gtsam::Matrix3 rotation_matrix = s.eigenvectors().real();

    // ensure rotation is right-handed
    // 函数检查旋转矩阵是否是右手坐标系，并将其转换为右手坐标系（如果不是）。
    if (!(fabs(1.0 - rotation_matrix.determinant()) < 1e-8))
    {
      rotation_matrix *= -1.0 * gtsam::Matrix3::Identity();
    }
    gtsam::Rot3 rotation(rotation_matrix);
    // 返回一个受约束的对偶二次曲面对象，其中包含旋转矩阵、平移向量和双曲面的形状。
    return ConstrainedDualQuadric(rotation, translation, shape);
  }

  /* ************************************************************************* */
  // 用于计算 受约束对偶二次曲面对象 的4x4矩阵表示Q。这个函数返回一个gtsam::Matrix44对象。
  // 可以选择同时计算Q的雅可比矩阵dQ_dq，大小为16x9
  gtsam::Matrix44 ConstrainedDualQuadric::matrix(
      gtsam::OptionalJacobian<16, 9> dQ_dq) const
  {
    // 其中Z是代表平移和旋转的4x4矩阵。
    gtsam::Matrix44 Z = pose_.matrix();
    // Qc是对角线矩阵，表示椭球的形状。
    // @ 论文中的计算式(2) @
    gtsam::Matrix44 Qc = (gtsam::Vector4() << (radii_).array().pow(2), -1.0)
                             .finished()
                             .asDiagonal();
    // 这个函数的实现首先将受约束对偶二次曲面表示为Q = Z*Qc*Z^T的形式，
    // @ 论文中的计算式(1) @
    gtsam::Matrix44 Q = Z * Qc * Z.transpose();

    // 计算Q的雅可比矩阵。
    if (dQ_dq)
    {
      // 在计算Z的导数时，该函数使用utils::matrix(pose_, dZ_dx)计算旋转部分的导数，
      // 使用dZ_dq.block(0, 0, 16, 6) = dZ_dx将其与平移部分的导数组合在一起。
      Eigen::Matrix<double, 16, 6> dZ_dx;
      utils::matrix(pose_, dZ_dx); // NOTE: this will recalculate pose.matrix
      Eigen::Matrix<double, 16, 9> dZ_dq = gtsam::Matrix::Zero(16, 9);
      dZ_dq.block(0, 0, 16, 6) = dZ_dx;

      // 对于Qc，这个函数根据radii_数组创建一个对角线矩阵，其中radii_数组包含三个表示椭球形状的半轴长度。
      // 在计算Qc对参数的导数时，该函数计算每个半轴长度对应的导数并在dQc_dq矩阵中进行排列。
      Eigen::Matrix<double, 16, 9> dQc_dq = gtsam::Matrix::Zero(16, 9);
      dQc_dq(0, 6) = 2.0 * radii_(0);
      dQc_dq(5, 7) = 2.0 * radii_(1);
      dQc_dq(10, 8) = 2.0 * radii_(2);

      // 使用utils::TVEC(4, 4)计算16x16的矩阵T44，
      // 使用kron函数将多个矩阵进行Kronecker积运算，最终得到dQ_dq的值。
      using utils::kron;
      static gtsam::Matrix4 I44 = gtsam::Matrix::Identity(4, 4);
      static Eigen::Matrix<double, 16, 16> T44 = utils::TVEC(4, 4);
      *dQ_dq = kron(I44, Z * Qc) * T44 * dZ_dq +
               kron(Z, I44) * (kron(I44, Z) * dQc_dq + kron(Qc, I44) * dZ_dq);
    }
    return Q;
  }

  /* ************************************************************************* */
  // 返回一个经过归一化的 受约束对偶二次曲面 对应的 4x4 的矩阵
  // @ 论文中的计算式(10) @
  gtsam::Matrix44 ConstrainedDualQuadric::normalizedMatrix(void) const
  {
    gtsam::Matrix44 Q = this->matrix();
    return Q / Q(3, 3);
  }

  /* ************************************************************************* */
  // TODO: vectorize：代码实现需要使用向量化操作。
  // 向量化是指将标量操作转换为向量或矩阵操作，以便利用现代处理器的 SIMD（Single Instruction, Multiple Data）指令集提高运行效率。
  // 因此，向量化通常可以加快代码的执行速度。

  // 这段代码实现了获取 受约束对偶二次曲面 的边界框的函数。
  AlignedBox3 ConstrainedDualQuadric::bounds() const
  {
    // 首先调用了ConstrainedDualQuadric::matrix()函数获取对象的 4x4 的矩阵表示 dE
    gtsam::Matrix44 dE = this->matrix();
    // 然后计算了(xmin, xmax, ymin, ymax, zmin, zmax)
    double x_min =
        (dE(0, 3) + std::sqrt(dE(0, 3) * dE(0, 3) - (dE(0, 0) * dE(3, 3)))) /
        dE(3, 3);
    double y_min =
        (dE(1, 3) + std::sqrt(dE(1, 3) * dE(1, 3) - (dE(1, 1) * dE(3, 3)))) /
        dE(3, 3);
    double z_min =
        (dE(2, 3) + std::sqrt(dE(2, 3) * dE(2, 3) - (dE(2, 2) * dE(3, 3)))) /
        dE(3, 3);
    double x_max =
        (dE(0, 3) - std::sqrt(dE(0, 3) * dE(0, 3) - (dE(0, 0) * dE(3, 3)))) /
        dE(3, 3);
    double y_max =
        (dE(1, 3) - std::sqrt(dE(1, 3) * dE(1, 3) - (dE(1, 1) * dE(3, 3)))) /
        dE(3, 3);
    double z_max =
        (dE(2, 3) - std::sqrt(dE(2, 3) * dE(2, 3) - (dE(2, 2) * dE(3, 3)))) /
        dE(3, 3);

    // 将这六个最小和最大值存储在一个 AlignedBox3 对象中
    return AlignedBox3((gtsam::Vector6() << std::min(x_min, x_max),
                        std::max(x_min, x_max), std::min(y_min, y_max),
                        std::max(y_min, y_max), std::min(z_min, z_max),
                        std::max(z_min, z_max))
                           .finished());
  }

  /* ************************************************************************* */
  // 这个函数判断给定的相机姿态是否在受限双二次曲面后面，即相机是否在曲面的“背面”。
  // 函数中首先将受限双二次曲面的位姿变换到给定相机姿态下，然后判断该变换后的曲面位姿的 z 轴在相机坐标系中的方向是否为负。
  bool ConstrainedDualQuadric::isBehind(const gtsam::Pose3 &cameraPose) const
  {
    gtsam::Pose3 rpose = cameraPose.between(this->pose());
    if (rpose.z() < 0.0)
    {
      return true;
    }
    return false;
  }

  /* ************************************************************************* */
  // 这段代码实现了判断一个相机是否在双重二次曲面的内部。
  // 它首先将相机的位置转换为一个齐次坐标向量，并计算它与双重二次曲面的距离。
  // 如果该距离小于或等于零，则相机在双重二次曲面内部，返回true。否则，相机在曲面外部，返回false。
  bool ConstrainedDualQuadric::contains(const gtsam::Pose3 &cameraPose) const
  {
    gtsam::Vector4 cameraPoint =
        (gtsam::Vector4() << cameraPose.translation(), 1.0).finished();
    double pointError =
        cameraPoint.transpose() * this->matrix().inverse() * cameraPoint;
    if (pointError <= 0.0)
    {
      return true;
    }
    return false;
  }

  /* ************************************************************************* */
  // 用于将一个包含 $9$ 个元素的向量 $v$ 映射为一个 ConstrainedDualQuadric 对象。
  // 该函数通过前6个元素构建一个 gtsam::Pose3 对象，用后3个元素构建一个 gtsam::Vector3 对象，
  // 然后用这两个对象构造一个 ConstrainedDualQuadric 对象并返回。
  ConstrainedDualQuadric ConstrainedDualQuadric::Retract(
      const gtsam::Vector9 &v)
  {
    // gtsam::Pose3::Retract 作用是将一个向量映射回其所属的流形
    // v.head<6>() 向量属于 ConstrainedDualQuadric 类的流形
    gtsam::Pose3 pose = gtsam::Pose3::Retract(v.head<6>());
    gtsam::Vector3 radii = v.tail<3>();
    return ConstrainedDualQuadric(pose, radii);
  }

  /* ************************************************************************* */
  // 用于将一个ConstrainedDualQuadric映射为一个 包含 $9$ 个元素的向量 $v$。
  // ConstrainedDualQuadric::Retract的对应函数
  gtsam::Vector9 ConstrainedDualQuadric::LocalCoordinates(
      const ConstrainedDualQuadric &q)
  {
    gtsam::Vector9 v = gtsam::Vector9::Zero();
    v.head<6>() = gtsam::Pose3::LocalCoordinates(q.pose_);
    v.tail<3>() = q.radii_;
    return v;
  }

  /* ************************************************************************* */
  // 接受一个 $9$ 维向量作为输入，对该类进行更新并返回更新后的结果。
  // 首先，函数将输入向量的前 $6$ 个元素提取出来，调用 pose_ 对象的成员函数 retract 进行更新，得到更新后的 pose。
  // 接下来，函数将输入向量的后 $3$ 个元素提取出来，与当前 radii_ 相加，得到更新后的 radii。
  // 最后，函数构造并返回更新后的 ConstrainedDualQuadric 对象。
  ConstrainedDualQuadric ConstrainedDualQuadric::retract(
      const gtsam::Vector9 &v) const
  {
    gtsam::Pose3 pose = pose_.retract(v.head<6>());
    gtsam::Vector3 radii = radii_ + v.tail<3>();
    return ConstrainedDualQuadric(pose, radii);
  }

  /* ************************************************************************* */
  // 这段代码实现了计算 受限对偶二次曲面 相对于另一个对象的本地坐标的功能。
  // 具体来说，给定两个实例this和other，该方法返回一个gtsam::Vector9对象，
  // 其中前6个元素是this.pose_相对于other.pose_的本地坐标，
  // 后3个元素是other.radii_相对于this.radii_的差异。
  gtsam::Vector9 ConstrainedDualQuadric::localCoordinates(
      const ConstrainedDualQuadric &other) const
  {
    gtsam::Vector9 v = gtsam::Vector9::Zero();
    v.head<6>() = pose_.localCoordinates(other.pose_);
    v.tail<3>() = other.radii_ - radii_;
    return v;
  }

  /* ************************************************************************* */
  void ConstrainedDualQuadric::print(const std::string &s) const
  {
    cout << s;
    cout << this->matrix() << endl;
  }

  /* ************************************************************************* */
  bool ConstrainedDualQuadric::equals(const ConstrainedDualQuadric &other,
                                      double tol) const
  {
    return this->normalizedMatrix().isApprox(other.normalizedMatrix(), tol);
  }

  /* ************************************************************************* */
  // 这段代码是将ConstrainedDualQuadric对象添加到GTSAM中的Values对象中，并使用给定的密钥k关联它们。
  // 在GTSAM库中，Values类是一个关联了密钥和值的容器，它被用于管理优化变量和估计结果。
  // 这个函数的目的是将ConstrainedDualQuadric对象添加到Values对象中，以便可以在GTSAM优化中使用它。
  void ConstrainedDualQuadric::addToValues(gtsam::Values &v,
                                           const gtsam::Key &k)
  {
    v.insert(k, *this);
  }

  /* ************************************************************************* */
  // 这段代码实现了从一个 gtsam::Values 对象中获取一个特定的 gtsam::Key 对应的 ConstrainedDualQuadric 对象。
  // 其中 gtsam::Values 是一个类似于字典的对象，它将每个 gtsam::Key 映射到对应的值。
  // 在这里，我们通过给定的 gtsam::Key，从 gtsam::Values 中获取对应的 ConstrainedDualQuadric 对象。
  ConstrainedDualQuadric ConstrainedDualQuadric::getFromValues(
      const gtsam::Values &v, const gtsam::Key &k)
  {
    return v.at<ConstrainedDualQuadric>(k);
  }

} // namespace gtsam_quadrics
