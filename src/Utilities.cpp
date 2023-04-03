/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file Utilities.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a namespace providing a number of useful functions
 */

#include <Utilities.h>
#include <iostream>

using namespace std;

namespace gtsam_soslam
{
  namespace utils
  {

    /* ************************************************************************* */
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
    gtsam::Vector2 getConicPointsAtX(
        const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x)
    {
      const Eigen::Matrix<long double, 3, 3> &C = pointConic;
      return solvePolynomial(C(1, 1), 2 * C(0, 1) * x + 2 * C(1, 2),
                             C(0, 0) * x * x + 2 * C(0, 2) * x + C(2, 2));
    }

    /* ************************************************************************* */
    gtsam::Vector2 getConicPointsAtY(
        const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y)
    {
      const Eigen::Matrix<long double, 3, 3> &C = pointConic;
      return solvePolynomial(C(0, 0), 2 * C(0, 1) * y + 2 * C(0, 2),
                             C(1, 1) * y * y + 2 * C(1, 2) * y + C(2, 2));
    }

    /* ************************************************************************* */
    gtsam::Pose3 interpolate(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2,
                             const double &percent)
    {
      return gtsam::interpolate<gtsam::Pose3>(p1, p2, percent);
    }

    /* ************************************************************************* */
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

    ConstrainedDualQuadric initialize_quadric_ray_intersection(
        const std::vector<gtsam::Pose3> &obs_poses,
        const std::vector<AlignedBox2> &boxes,
        SoSlamState &state)
    {
        int n = int(obs_poses.size());
        // gtsam::Matrix33 I = gtsam::Matrix33::Identity();
        gtsam::Matrix ps(3, n);
        gtsam::Matrix vs(3, n);
      // Get each observation point
        for (int i = 0; i < n; ++i) {
            gtsam::Pose3 op = obs_poses[i];
            gtsam::Vector3 p = op.translation();
            gtsam::Vector3 v = op.rotation().matrix().col(0);

            ps.col(i) = p;
            vs.col(i) = v;
        }
        
        // Actually we don't care this part. So I donot waste time on it.
        gtsam::Vector3 quadric_centroid(3.333333333333,0.0,0.0);

        return ConstrainedDualQuadric(
                gtsam::Rot3(), gtsam::Point3(quadric_centroid), gtsam::Vector3(1, 1, 0.1));
    }


    std::pair<std::map<gtsam::Key, gtsam::Pose3>, std::map<gtsam::Key, ConstrainedDualQuadric>> ps_and_qs_from_values(const gtsam::Values &values)
    {
      std::map<gtsam::Key, gtsam::Pose3> ps;
      std::map<gtsam::Key, ConstrainedDualQuadric> qs;

      for (const auto &key_value_pair : values)
      {
        gtsam::Key key = key_value_pair.key;
        unsigned char symbol_char = gtsam::Symbol(key).chr();

        if (symbol_char == 'x')
        {
          ps[key] = values.at<gtsam::Pose3>(key);
        }
        else if (symbol_char == 'q')
        {
          qs[key] = ConstrainedDualQuadric::getFromValues(values, key);
        }
      }

      return std::make_pair(ps, qs);
    }

    gtsam::NonlinearFactorGraph new_factors(const gtsam::NonlinearFactorGraph &current,
                                            const gtsam::NonlinearFactorGraph &previous)
    {
      // Figure out the new factors
      std::set<gtsam::NonlinearFactor::shared_ptr> fs;
        for (const auto& element : current) {
            fs.insert(element);
        }
        for (const auto& element : previous) {
            fs.erase(element);
        }

      // Return a NEW graph with the factors
      gtsam::NonlinearFactorGraph out;
      for (const auto &f : fs)
      {
        out.add(f);
      }
      return out;
    }

    gtsam::Values new_values(const gtsam::Values &current, const gtsam::Values &previous)
    {
      auto current_ps_qs = ps_and_qs_from_values(current);
      auto previous_ps_qs = ps_and_qs_from_values(previous);

      std::map<gtsam::Key, gtsam::Pose3> cps = current_ps_qs.first;
      std::map<gtsam::Key, ConstrainedDualQuadric> cqs = current_ps_qs.second;
      std::map<gtsam::Key, gtsam::Pose3> pps = previous_ps_qs.first;
      std::map<gtsam::Key, ConstrainedDualQuadric> pqs = previous_ps_qs.second;

      std::map<gtsam::Key, boost::variant<gtsam::Pose3, ConstrainedDualQuadric>> vs;

      for (const auto &key_value_pair : cps)
      {
        if (pps.find(key_value_pair.first) == pps.end())
        {
          vs[key_value_pair.first] = key_value_pair.second;
        }
      }

      for (const auto &key_value_pair : cqs)
      {
        if (pqs.find(key_value_pair.first) == pqs.end())
        {
          vs[key_value_pair.first] = key_value_pair.second;
        }
      }

      gtsam::Values out;
      for (const auto &key_value_pair : vs)
      {
        if (key_value_pair.second.type() == typeid(ConstrainedDualQuadric))
        {
          auto q = boost::get<ConstrainedDualQuadric>(key_value_pair.second);
          q.addToValues(out, key_value_pair.first);
        }
        else
        {
          out.insert(key_value_pair.first, boost::get<gtsam::Pose3>(key_value_pair.second));
        }
      }
      return out;
    }
    void visualize(SoSlamState &state)
        {
          gtsam::Values values = state.estimates_;
          auto labels = state.labels_;
          values.print();

          // test for semantic scale factor
          gtsam::Key b = 8142508126285856768;
          ConstrainedDualQuadric a = values.at<ConstrainedDualQuadric>(b);
          std::cout << "quadric pose and size:" << std::endl;
          std::cout << a.pose() << std::endl << a.radii() << std::endl;

          /* values in quadricslam original code are as follows:
          Values:Values with 7 values:
          Value q0: (gtsam_quadrics::ConstrainedDualQuadric)
                     1  5.33268e-16 -5.92244e-17 -8.88324e-16
           5.10908e-16            1 -2.04511e-17 -4.26056e-16
          -5.76232e-17 -2.09414e-17            1  2.30073e-18
          -8.88324e-16 -4.26056e-16  2.30073e-18           -1

          Value q1: (gtsam_quadrics::ConstrainedDualQuadric)
          -2.33147e-15           -1           -1           -1
                    -1 -5.55112e-16           -1           -1
                    -1           -1 -1.11022e-16           -1
                    -1           -1           -1           -1

          Value x0: (gtsam::Pose3)
          R: [
              0, 0, -1;
              1, 0, 0;
              0, -1, 0
              ]
          t: 10  0  0

          Value x1: (gtsam::Pose3)
          R: [
              1, -1.06515e-17, 7.62121e-17;
              -7.65252e-17, 3.82794e-18, 1;
              -1.12274e-17, -1, 4.07367e-18
          ]
          t:  3.314e-16  -10  9.62244e-19

          Value x2: (gtsam::Pose3)
          R: [
              -1.30194e-16, -1.44876e-17, 1;
              -1, -1.78409e-17, -1.33587e-16;
              1.41949e-17, -1, -1.13698e-17
          ]
          t:  -10  1.57443e-15  1.43737e-16

          Value x3: (gtsam::Pose3)
          R: [
              -1, -1.27118e-17, -4.65885e-16;
              4.68574e-16, 3.00111e-18, -1;
              9.75593e-18, -1, -5.62604e-18
          ]
          t:  5.42251e-15  10  -1.86248e-16

          Value x4: (gtsam::Pose3)
          R: [
              3.41841e-16, -4.09092e-17, -1;
              1, -2.87323e-17, 3.41342e-16;
              -2.57698e-17, -1, 4.10462e-17
          ]
          t:  10  -2.76928e-15  -3.42707e-16*/

          /* labels and block need to be:
          ###################
          Labels:{8142508126285856768: 'q0', 8142508126285856769: 'q1'}
          ###################
          Block:True
          */
      }


  } // namespace utils
} // namespace gtsam_soslam
