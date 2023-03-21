/* ----------------------------------------------------------------------------
The code provides the following functions:

solvePolynomial(const double &a, const double &b, const double &c): This function calculates the roots of a quadratic polynomial with coefficients a, b, and c and returns the roots as a 2D vector.

getConicPointsAtX(const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x): This function calculates the y-coordinates of the two points at which a conic intersects a vertical line at a given x-coordinate. The function takes a 3x3 matrix representing the conic, and a double representing the x-coordinate.

getConicPointsAtY(const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y): This function calculates the x-coordinates of the two points at which a conic intersects a horizontal line at a given y-coordinate. The function takes a 3x3 matrix representing the conic, and a double representing the y-coordinate.

interpolate(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2, const double &percent): This function linearly interpolates between two Pose3 objects p1 and p2 using the given percentage value percent. It returns the interpolated Pose3.

matrix(const gtsam::Pose3 &pose, gtsam::OptionalJacobian<16, 6> H): This function returns the 4x4 matrix representation of a Pose3 object, with an optional Jacobian matrix H.

kron(const gtsam::Matrix m1, const gtsam::Matrix m2): This function computes the Kronecker product of two matrices m1 and m2 and returns the result as a new matrix.

 * -------------------------------------------------------------------------- */

/**
 * @file Utilities.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a namespace providing a number of useful functions
 */

#include <gtsam_quadrics/base/Utilities.h>

#include <iostream>

using namespace std;

namespace gtsam_quadrics
{
  namespace utils
  {

    /* ************************************************************************* */
    // 这个函数是用来求解方程 $ax^2+bx+c=0$ 的两个根（或者说解）。
    // 它的输入是三个实数 $a$、$b$ 和 $c$，输出是一个长度为 2 的向量，表示方程的两个解。
    gtsam::Vector2 solvePolynomial(const double &a, const double &b,
                                   const double &c)
    {
      // calculate polynomial discrimenant
      double disc = b * b - 4.0 * a * c;

      if (disc < 1e-10)
      {
        disc = 0.0;
      }

      // throw exception if imaginary results
      if (disc < 0.0)
      {
        stringstream ss;
        ss << "poly solving failed, disc: " << disc << endl;
        throw std::runtime_error(ss.str());
      }

      // calculate and return roots
      double root1 = (-b + std::sqrt(disc)) / (2.0 * a);
      double root2 = (-b - std::sqrt(disc)) / (2.0 * a);
      return gtsam::Vector2(root1, root2);
    }

    /* ************************************************************************* */
    // 这个函数用来求解二次曲线在 $x$ 坐标为 $x$ 时的两个点。
    // 它的输入是一个 $3\times 3$ 的矩阵，表示一个二次曲线（即点和曲线的关系，详见二次曲线）。
    // 它的输出是一个长度为 2 的向量，表示二次曲线在 $x$ 坐标为 $x$ 时的两个点的 $y$ 坐标。
    gtsam::Vector2 getConicPointsAtX(
        const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x)
    {
      const Eigen::Matrix<long double, 3, 3> &C = pointConic;
      return solvePolynomial(C(1, 1), 2 * C(0, 1) * x + 2 * C(1, 2),
                             C(0, 0) * x * x + 2 * C(0, 2) * x + C(2, 2));
    }

    /* ************************************************************************* */
    // 这个函数用来求解二次曲线在 $y$ 坐标为 $y$ 时的两个点。
    // 它的输入和输出分别与getConicPointsAtX相同，只是在输入矩阵中代表 $x$ 和 $y$ 的行和列互换了。
    gtsam::Vector2 getConicPointsAtY(
        const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y)
    {
      const Eigen::Matrix<long double, 3, 3> &C = pointConic;
      return solvePolynomial(C(0, 0), 2 * C(0, 1) * y + 2 * C(0, 2),
                             C(1, 1) * y * y + 2 * C(1, 2) * y + C(2, 2));
    }

    /* ************************************************************************* */
    // 这个函数用来计算两个位姿之间的插值。它的输入是两个位姿 $p1$ 和 $p2$，以及一个实数 percent，
    // 表示在两个位姿之间插值的位置，取值范围为 $[0,1]$，当 percent=0 时返回 $p1$，当 percent=1 时返回 $p2$。
    // 它的输出是一个新的位姿，表示两个位姿之间插值的结果。
    gtsam::Pose3 interpolate(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2,
                             const double &percent)
    {
      return gtsam::interpolate<gtsam::Pose3>(p1, p2, percent);
    }

    /* ************************************************************************* */
    // 这个函数用来计算一个位姿的齐次变换矩阵（详见齐次坐标系）。它的输入是一个位姿 $pose$，
    // 以及一个可选的输出参数 $H$，表示齐次变换矩阵对位姿的一阶导数。
    // 如果不需要求导数，可以将 $H$ 设置为 boost::none。它的输出是一个 $4\times 4$ 的齐次变换矩阵。
    gtsam::Matrix44 matrix(const gtsam::Pose3 &pose,
                           gtsam::OptionalJacobian<16, 6> H)
    {
      gtsam::Matrix44 poseMatrix = pose.matrix();

      if (H)
      {
        H->setZero();
        (*H)(4, 0) = poseMatrix(0, 2);
        (*H)(5, 0) = poseMatrix(1, 2);
        (*H)(6, 0) = poseMatrix(2, 2);
        (*H)(8, 0) = -poseMatrix(0, 1);
        (*H)(9, 0) = -poseMatrix(1, 1);
        (*H)(10, 0) = -poseMatrix(2, 1);

        (*H)(0, 1) = -poseMatrix(0, 2);
        (*H)(1, 1) = -poseMatrix(1, 2);
        (*H)(2, 1) = -poseMatrix(2, 2);
        (*H)(8, 1) = poseMatrix(0, 0);
        (*H)(9, 1) = poseMatrix(1, 0);
        (*H)(10, 1) = poseMatrix(2, 0);

        (*H)(0, 2) = poseMatrix(0, 1);
        (*H)(1, 2) = poseMatrix(1, 1);
        (*H)(2, 2) = poseMatrix(2, 1);
        (*H)(4, 2) = -poseMatrix(0, 0);
        (*H)(5, 2) = -poseMatrix(1, 0);
        (*H)(6, 2) = -poseMatrix(2, 0);

        (*H)(12, 3) = poseMatrix(0, 0);
        (*H)(13, 3) = poseMatrix(1, 0);
        (*H)(14, 3) = poseMatrix(2, 0);

        (*H)(12, 4) = poseMatrix(0, 1);
        (*H)(13, 4) = poseMatrix(1, 1);
        (*H)(14, 4) = poseMatrix(2, 1);

        (*H)(12, 5) = poseMatrix(0, 2);
        (*H)(13, 5) = poseMatrix(1, 2);
        (*H)(14, 5) = poseMatrix(2, 2);
      }
      return poseMatrix;
    }

    /* ************************************************************************* */
    // 用于计算两个矩阵的 Kronecker 乘积。该函数的两个参数是两个 gtsam::Matrix 类型的矩阵。
    // 该函数的返回值是一个新的矩阵，该矩阵的行数为第一个矩阵的行数乘以第二个矩阵的行数，列数为第一个矩阵的列数乘以第二个矩阵的列数。
    gtsam::Matrix kron(const gtsam::Matrix m1, const gtsam::Matrix m2)
    {
      gtsam::Matrix m3(m1.rows() * m2.rows(), m1.cols() * m2.cols());

      for (int j = 0; j < m1.cols(); j++)
      {
        for (int i = 0; i < m1.rows(); i++)
        {
          m3.block(i * m2.rows(), j * m2.cols(), m2.rows(), m2.cols()) =
              m1(i, j) * m2;
        }
      }
      return m3;
    }

    /* ************************************************************************* */
    // 这个函数的作用是返回一个大小为m*n的置换矩阵T，其中m和n是输入参数。
    // 置换矩阵是一个方阵，其元素仅在矩阵的一条对角线上为1，其余元素为0。
    gtsam::Matrix TVEC(const int m, const int n)
    {
      gtsam::Matrix T(m * n, m * n);
      for (int j = 0; j < m * n; j++)
      {
        for (int i = 0; i < m * n; i++)
        {
          if ((j + 1) == (1 + (m * ((i + 1) - 1)) -
                          ((m * n - 1) * floor(((i + 1) - 1) / n))))
          {
            T(i, j) = 1;
          }
          else
          {
            T(i, j) = 0;
          }
        }
      }
      return T;
    }

  } // namespace utils
} // namespace gtsam_quadrics
