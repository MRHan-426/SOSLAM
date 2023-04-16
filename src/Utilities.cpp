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
using namespace Eigen;

namespace gtsam_soslam {
    namespace utils {

        /* ************************************************************************* */
        gtsam::Vector2 solvePolynomial(const double &a, const double &b,
                                       const double &c) {
            // calculate polynomial discrimenant
            double disc = b * b - 4.0 * a * c;

            if (disc < 1e-10) {
                disc = 0.0;
            }

            // throw exception if imaginary results
            if (disc < 0.0) {
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
                const Eigen::Matrix<long double, 3, 3> &pointConic, const double &x) {
            const Eigen::Matrix<long double, 3, 3> &C = pointConic;
            return solvePolynomial(C(1, 1), 2 * C(0, 1) * x + 2 * C(1, 2),
                                   C(0, 0) * x * x + 2 * C(0, 2) * x + C(2, 2));
        }

        /* ************************************************************************* */
        gtsam::Vector2 getConicPointsAtY(
                const Eigen::Matrix<long double, 3, 3> &pointConic, const double &y) {
            const Eigen::Matrix<long double, 3, 3> &C = pointConic;
            return solvePolynomial(C(0, 0), 2 * C(0, 1) * y + 2 * C(0, 2),
                                   C(1, 1) * y * y + 2 * C(1, 2) * y + C(2, 2));
        }

        /* ************************************************************************* */
        gtsam::Pose3 interpolate(const gtsam::Pose3 &p1, const gtsam::Pose3 &p2,
                                 const double &percent) {
            return gtsam::interpolate<gtsam::Pose3>(p1, p2, percent);
        }

        /* ************************************************************************* */
        gtsam::Matrix44 matrix(const gtsam::Pose3 &pose,
                               gtsam::OptionalJacobian<16, 6> H) {
            gtsam::Matrix44 poseMatrix = pose.matrix();

            if (H) {
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
        gtsam::Matrix kron(const gtsam::Matrix &m1, const gtsam::Matrix &m2) {
            gtsam::Matrix m3(m1.rows() * m2.rows(), m1.cols() * m2.cols());

            for (int j = 0; j < m1.cols(); j++) {
                for (int i = 0; i < m1.rows(); i++) {
                    m3.block(i * m2.rows(), j * m2.cols(), m2.rows(), m2.cols()) =
                            m1(i, j) * m2;
                }
            }
            return m3;
        }

        /* ************************************************************************* */
        gtsam::Matrix TVEC(int m, int n) {
            gtsam::Matrix T(m * n, m * n);
            for (int j = 0; j < m * n; j++) {
                for (int i = 0; i < m * n; i++) {
                    if ((j + 1) == (1 + (m * ((i + 1) - 1)) -
                                    ((m * n - 1) * floor(((i + 1) - 1) / n)))) {
                        T(i, j) = 1;
                    } else {
                        T(i, j) = 0;
                    }
                }
            }
            return T;
        }

        ConstrainedDualQuadric initialize_quadric_ray_intersection(
                const std::vector<gtsam::Pose3> &obs_poses,
                const std::vector<AlignedBox2> &boxes,
                SoSlamState &state) {
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
            gtsam::Vector3 quadric_centroid(3.333333333333, 0.0, 0.0);

            return ConstrainedDualQuadric(
                    gtsam::Rot3(), gtsam::Point3(quadric_centroid), gtsam::Vector3(1, 1, 0.1));
        }

        ConstrainedDualQuadric initialize_with_ssc_psc_bbs_syc(
                const BoundingBoxFactor &bbs,
                const SemanticScaleFactor &ssc,
                const PlaneSupportingFactor &psc,
                const SymmetryFactor &syc,
                const gtsam::Pose3 &camera_pose

        ) {

            gtsam::NonlinearFactorGraph sub_graph;
            gtsam::Values initial_estimate;
            auto noise_prior = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Zero());
            sub_graph.add(gtsam::PriorFactor<gtsam::Pose3>(bbs.poseKey(), camera_pose, noise_prior));
            sub_graph.add(bbs);
            sub_graph.add(ssc);
            sub_graph.add(psc);

            //      sub_graph.add(syc);
            std::cout << "###############################" << std::endl;
            std::cout << "BALL: " << bbs.objectKey() << std::endl;
            gtsam::Point3 localPoint(0, 0, 2);
            gtsam::Point3 globalPoint = camera_pose.transformFrom(localPoint);
            // set a prior quadric
            ConstrainedDualQuadric quadric(gtsam::Pose3(gtsam::Rot3(), globalPoint), gtsam::Vector3(1, 1, 1));

            //        ConstrainedDualQuadric quadric;
            initial_estimate.insert(bbs.objectKey(), quadric);
            initial_estimate.insert(bbs.poseKey(), camera_pose);

            gtsam::LevenbergMarquardtParams params;
            gtsam::LevenbergMarquardtOptimizer optimizer(sub_graph, initial_estimate, params);
            auto result = optimizer.optimize();

            ConstrainedDualQuadric initial_quadric = result.at<ConstrainedDualQuadric>(bbs.objectKey());
            gtsam::Pose3 initial_pose = result.at<gtsam::Pose3>(bbs.poseKey());
            //        cout<<"camera pose"<<endl<<camera_pose<<endl;
            //        cout<<"inited pose"<<endl<<initial_pose<<endl;
            //        std::cout << initial_quadric.pose() << std::endl << initial_quadric.radii() << std::endl;
            return initial_quadric;
        }

        std::pair<std::map<gtsam::Key, gtsam::Pose3>, std::map<gtsam::Key, ConstrainedDualQuadric>>
        ps_and_qs_from_values(const gtsam::Values &values) {
            std::map<gtsam::Key, gtsam::Pose3> ps;
            std::map<gtsam::Key, ConstrainedDualQuadric> qs;

            for (const auto &key_value_pair: values) {
                gtsam::Key key = key_value_pair.key;
                unsigned char symbol_char = gtsam::Symbol(key).chr();

                if (symbol_char == 'x') {
                    ps[key] = values.at<gtsam::Pose3>(key);
                } else if (symbol_char == 'q') {
                    qs[key] = ConstrainedDualQuadric::getFromValues(values, key);
                }
            }

            return std::make_pair(ps, qs);
        }

        gtsam::NonlinearFactorGraph new_factors(const gtsam::NonlinearFactorGraph &current,
                                                const gtsam::NonlinearFactorGraph &previous) {
            // Figure out the new factors
            std::set<gtsam::NonlinearFactor::shared_ptr> fs;
            for (const auto &element: current) {
                fs.insert(element);
            }
            for (const auto &element: previous) {
                fs.erase(element);
            }

            // Return a NEW graph with the factors
            gtsam::NonlinearFactorGraph out;
            for (const auto &f: fs) {
                out.add(f);
            }
            return out;
        }

        gtsam::Values new_values(const gtsam::Values &current, const gtsam::Values &previous) {
            auto current_ps_qs = ps_and_qs_from_values(current);
            auto previous_ps_qs = ps_and_qs_from_values(previous);

            std::map<gtsam::Key, gtsam::Pose3> cps = current_ps_qs.first;
            std::map<gtsam::Key, ConstrainedDualQuadric> cqs = current_ps_qs.second;
            std::map<gtsam::Key, gtsam::Pose3> pps = previous_ps_qs.first;
            std::map<gtsam::Key, ConstrainedDualQuadric> pqs = previous_ps_qs.second;

            std::map<gtsam::Key, boost::variant<gtsam::Pose3, ConstrainedDualQuadric>> vs;

            for (const auto &key_value_pair: cps) {
                if (pps.find(key_value_pair.first) == pps.end()) {
                    vs[key_value_pair.first] = key_value_pair.second;
                }
            }

            for (const auto &key_value_pair: cqs) {
                if (pqs.find(key_value_pair.first) == pqs.end()) {
                    vs[key_value_pair.first] = key_value_pair.second;
                }
            }

            gtsam::Values out;
            for (const auto &key_value_pair: vs) {
                if (key_value_pair.second.type() == typeid(ConstrainedDualQuadric)) {
                    auto q = boost::get<ConstrainedDualQuadric>(key_value_pair.second);
                    q.addToValues(out, key_value_pair.first);
                } else {
                    out.insert(key_value_pair.first, boost::get<gtsam::Pose3>(key_value_pair.second));
                }
            }
            return out;
        }

        double area(const gtsam::Vector4 &bounds) {
            return (bounds[2] - bounds[0]) * (bounds[3] - bounds[1]);
        }

        double iou(const AlignedBox2 &a, const AlignedBox2 &b) {
            gtsam::Vector4 intersection_bounds;
            intersection_bounds[0] = std::min(a.xmin(), b.xmin());
            intersection_bounds[1] = std::min(a.ymin(), b.ymin());
            intersection_bounds[2] = std::max(a.xmax(), b.xmax());
            intersection_bounds[3] = std::max(a.ymax(), b.ymax());

            double int_area = area(intersection_bounds);
            double union_area = area(a.vector()) + area(b.vector()) - int_area;
            return int_area / union_area;
        }

        void visualize(SoSlamState &state) {
            gtsam::Values values = state.estimates_;
            auto labels = state.labels_;
            //        std::vector<double> ious = evaluate::iou_evaluation(state);
            //        int i = 1;
            //        for (auto iou : ious){
            //            std::cout << "quadric " << i << "IOU = " << iou << std::endl;
            //            i++;
            //        }
        }
        Matrix4d getTransformFromVector(VectorXd& pose)
        {
            if( pose.rows() == 7)
            {
                // x y z qx qy qz qw
                Matrix4d homogeneous_matrix;
                Quaterniond _r(pose(6),pose(3),pose(4),pose(5));   // w x y z
                homogeneous_matrix.setIdentity();
                homogeneous_matrix.block(0,0,3,3) = _r.toRotationMatrix();
                homogeneous_matrix.col(3).head(3) = pose.head(3);

                return homogeneous_matrix;
            }
        }
        PointCloud* pclToQuadricPointCloudPtr(PointCloudPCL::Ptr &pCloud)
        {
            PointCloud* cloudPtr = new PointCloud;
            PointCloud& cloud = *cloudPtr;
            int num = pCloud->points.size();
            for(int i=0;i<num;i++){
                PointXYZRGB p;
                PointT pT = pCloud->points[i];
                p.r = pT.r;
                p.g = pT.g;
                p.b = pT.b;

                p.x = pT.x;
                p.y = pT.y;
                p.z = pT.z;
                cloud.push_back(p);
            }

            return cloudPtr;
        }
    } // namespace utils
} // namespace gtsam_soslam
