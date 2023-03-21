/* ----------------------------------------------------------------------------
这段代码主要是实现了：
  1.投影对偶二次曲面到图像的功能。
  2.将二维框投影到图像中并返回方程。

QuadricCamera::transformToImage() 
  函数将一个三维姿态和一个内参矩阵作为输入，返回一个 $3\times 4$ 的矩阵，表示将一个点从相机坐标系下变换到图像坐标系下的变换。它实际上是一个 $3\times 4$ 的变换矩阵，其中矩阵的前三列是内参矩阵，矩阵的第四列是姿态矩阵的逆矩阵的前三列与内参矩阵相乘的结果。

QuadricCamera::project() 
  函数将一个有约束的双重二次型（ConstrainedDualQuadric）、一个三维姿态和一个内参矩阵作为输入，返回一个双圆锥（DualConic），表示在图像中投影这个双重二次型的结果。它的计算方法是，先将输入的双重二次型和三维姿态进行“退化”操作（retract）得到双重二次型的变换矩阵 $Q$ 和姿态的变换矩阵 $\Xi$，然后计算图像坐标系下的投影矩阵 $P=K[I_{3\times 4}][\Xi]_{0:2,0:3}$，再根据公式 $C= PQP^\top$ 计算出双圆锥 $C$。同时，如果需要计算雅可比矩阵，可以得到双圆锥对输入双重二次型的雅可比矩阵和双圆锥对输入姿态的雅可比矩阵。

QuadricCamera::project() 
  函数的另一个重载版本将一个二维框（AlignedBox2）、一个三维姿态和一个内参矩阵作为输入，返回一个向量数组，表示将二维框的四条边在图像中投影后得到的平面方程（法向量 $a,b,c$ 和距离 $d$），其中每个向量都是一个四维向量 $[a, b, c, d]$。

 * -------------------------------------------------------------------------- */

/**
 * @file QuadricCamera.cpp
 * @date Apr 16, 2020
 * @author Lachlan Nicholson
 * @brief a class responsible for projecting quadrics
 */

#include <gtsam/base/numericalDerivative.h>
#include <gtsam_quadrics/base/Utilities.h>
#include <gtsam_quadrics/geometry/QuadricCamera.h>

namespace gtsam_quadrics
{

  /* ************************************************************************* */
  // 函数将一个三维姿态和一个相机内参矩阵作为输入，表示将一个点从相机坐标系下变换到图像坐标系下的变换。
  // 返回一个 3x4 的变换矩阵，前三列是内参矩阵，第四列是姿态矩阵的逆矩阵的前三列与内参矩阵相乘的结果。
  gtsam::Matrix34 QuadricCamera::transformToImage(
      const gtsam::Pose3 &pose,
      const boost::shared_ptr<gtsam::Cal3_S2> &calibration)
  {
    gtsam::Matrix3 image_T_camera = calibration->K();
    gtsam::Matrix4 camera_T_world = pose.inverse().matrix();
    gtsam::Matrix34 image_T_world =
        image_T_camera * (camera_T_world).block(0, 0, 3, 4);
    // Matrix34 image_T_world = image_T_camera * internal::I34 * camera_T_world;
    return image_T_world;
  }

  /* ************************************************************************* */
  // 函数将一个 ConstrainedDualQuadric、一个三维姿态和一个内参矩阵作为输入，返回一个对偶锥曲线（DualConic）。
  // 表示三维 ConstrainedDualQuadric 投影到图像得到的平面图形 dualConic。
  // 如果需要计算雅可比矩阵，可以得到 对偶锥曲线 对 对偶二次曲面 和对 输入姿态 的雅可比矩阵。
  DualConic QuadricCamera::project(
      const ConstrainedDualQuadric &quadric, const gtsam::Pose3 &pose,
      const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
      gtsam::OptionalJacobian<9, 9> dC_dq, gtsam::OptionalJacobian<9, 6> dC_dx)
  {
    // 先将对偶双曲线和三维姿态进行“退化”操作（retract）得到对偶二次曲面的变换矩阵 Q 和姿态的变换矩阵 Xi
    // first retract quadric and pose to compute dX:/dx and dQ:/dq
    gtsam::Matrix3 K = calibration->K();
    gtsam::Matrix4 Xi = pose.inverse().matrix();
    static gtsam::Matrix34 I34 = gtsam::Matrix::Identity(3, 4);

    // 然后计算图像坐标系下的投影矩阵
    gtsam::Matrix34 P = K * I34 * Xi;
    gtsam::Matrix4 Q = quadric.matrix();

    // 再根据公式 C= P*Q*P^T 计算出 对偶锥曲线(dualConic) C。
    gtsam::Matrix3 C = P * Q * P.transpose();
    DualConic dualConic(C);

    // 如果需要计算雅可比矩阵，可以得到 对偶锥曲线(dualConic) 对 对偶二次曲面 的雅可比矩阵。
    if (dC_dq)
    {
      Eigen::Matrix<double, 9, 16> dC_dQ = utils::kron(P, P);
      Eigen::Matrix<double, 16, 9> dQ_dq;
      quadric.matrix(dQ_dq); // NOTE: this recalculates quadric.matrix
      *dC_dq = dC_dQ * dQ_dq;
    }

    // 如果需要计算雅可比矩阵，可以得到 对偶锥曲线(dualConic) 对 输入姿态 的雅可比矩阵。
    if (dC_dx)
    {
      using utils::kron;
      static gtsam::Matrix33 I33 = gtsam::Matrix::Identity(3, 3);
      static gtsam::Matrix44 I44 = gtsam::Matrix::Identity(4, 4);
      Eigen::Matrix<double, 9, 12> dC_dP =
          kron(I33, P * Q) * utils::TVEC(3, 4) + kron(P * Q.transpose(), I33);
      Eigen::Matrix<double, 12, 16> dP_dXi = kron(I44, K * I34);
      Eigen::Matrix<double, 16, 16> dXi_dX = -kron(Xi.transpose(), Xi);
      Eigen::Matrix<double, 16, 6> dX_dx;
      utils::matrix(pose, dX_dx);
      *dC_dx = dC_dP * dP_dXi * dXi_dX * dX_dx;
    }

    return dualConic;
  }

  /* ************************************************************************* */
  // 将一个二维框（AlignedBox2）、一个三维姿态和一个内参矩阵作为输入，返回一个向量数组，
  // 表示将二维框的四条边在图像中投影后得到的平面方程（法向量 a,b,c 和距离 d），
  // 其中每个向量都是一个四维向量 [a, b, c, d]。
  std::vector<gtsam::Vector4> QuadricCamera::project(
      const AlignedBox2 &box, const gtsam::Pose3 &pose,
      const boost::shared_ptr<gtsam::Cal3_S2> &calibration)
  {
    std::vector<gtsam::Vector4> planes;
    for (auto line : box.lines())
    {
      gtsam::Vector4 plane =
          QuadricCamera::transformToImage(pose, calibration).transpose() * line;
      planes.push_back(plane);
    }
    return planes;
  }

} // namespace gtsam_quadrics
