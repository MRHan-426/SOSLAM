/* ----------------------------------------------------------------------------

 * QuadricSLAM Copyright 2020, ARC Centre of Excellence for Robotic Vision,
 Queensland University of Technology (QUT)
 * Brisbane, QLD 4000
 * All Rights Reserved
 * Authors: Lachlan Nicholson, et al. (see THANKS for the full author list)
 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file ConstrainedDualQuadric.cpp
 * @date Apr 14, 2020
 * @author Lachlan Nicholson
 * @brief a constrained dual quadric
 */
#include "../src/Polygon/Polygon.hpp"

#include <Utilities.h>
#include <AlignedBox2.h>
#include <ConstrainedDualQuadric.h>
#include <QuadricCamera.h>
#include <eigen3/Eigen/Eigenvalues>
#include <iostream>

using namespace std;

namespace gtsam_soslam {

/* ************************************************************************* */
    ConstrainedDualQuadric::ConstrainedDualQuadric() {
        pose_ = gtsam::Pose3();
        radii_ = gtsam::Vector3(1, 1, 1);
    }

/* ************************************************************************* */
    ConstrainedDualQuadric::ConstrainedDualQuadric(const gtsam::Matrix44 &dQ) {
        *this = ConstrainedDualQuadric::constrain(dQ);
    }

/* ************************************************************************* */
    ConstrainedDualQuadric ConstrainedDualQuadric::constrain(
            const gtsam::Matrix4 &dual_quadric) {

        // normalize if required
        gtsam::Matrix4 normalized_dual_quadric(dual_quadric);
        if (dual_quadric(3, 3) != 1.0) {
            normalized_dual_quadric = dual_quadric / dual_quadric(3, 3);
        }

        // extract translation
        gtsam::Point3 translation(normalized_dual_quadric.block(0, 3, 3, 1));

        // calculate the point quadric matrix
        gtsam::Matrix4 point_quadric = normalized_dual_quadric.inverse();
        gtsam::Matrix4 normalized_point_quadric = point_quadric;
        if (point_quadric(3, 3) != 1.0) {
            normalized_point_quadric = point_quadric / point_quadric(3, 3);
        }

        // extract shape
        auto lambdaa = normalized_point_quadric.block(0, 0, 3, 3).eigenvalues();
        gtsam::Vector3 shape =
                Eigen::sqrt(-1.0 * normalized_point_quadric.determinant() /
                            normalized_point_quadric.block(0, 0, 3, 3).determinant() *
                            1.0 / lambdaa.array())
                        .abs();

        // extract rotation
        Eigen::EigenSolver<Eigen::Matrix<double, 3, 3>> s(
                normalized_point_quadric.block(0, 0, 3, 3));
        gtsam::Matrix3 rotation_matrix = s.eigenvectors().real();

        // ensure rotation is right-handed
        if (!(fabs(1.0 - rotation_matrix.determinant()) < 1e-8)) {
            rotation_matrix *= -1.0 * gtsam::Matrix3::Identity();
        }
        gtsam::Rot3 rotation(rotation_matrix);

        return ConstrainedDualQuadric(rotation, translation, shape);
    }

/* ************************************************************************* */
    gtsam::Matrix44 ConstrainedDualQuadric::matrix(
            gtsam::OptionalJacobian<16, 9> dQ_dq) const {
        gtsam::Matrix44 Z = pose_.matrix();
        gtsam::Matrix44 Qc = (gtsam::Vector4() << (radii_).array().pow(2), -1.0)
                .finished()
                .asDiagonal();
        gtsam::Matrix44 Q = Z * Qc * Z.transpose();

        if (dQ_dq) {
            Eigen::Matrix<double, 16, 6> dZ_dx;
            utils::matrix(pose_, dZ_dx);  // NOTE: this will recalculate pose.matrix
            Eigen::Matrix<double, 16, 9> dZ_dq = gtsam::Matrix::Zero(16, 9);
            dZ_dq.block(0, 0, 16, 6) = dZ_dx;

            Eigen::Matrix<double, 16, 9> dQc_dq = gtsam::Matrix::Zero(16, 9);
            dQc_dq(0, 6) = 2.0 * radii_(0);
            dQc_dq(5, 7) = 2.0 * radii_(1);
            dQc_dq(10, 8) = 2.0 * radii_(2);

            using utils::kron;
            static gtsam::Matrix4 I44 = gtsam::Matrix::Identity(4, 4);
            static Eigen::Matrix<double, 16, 16> T44 = utils::TVEC(4, 4);
            *dQ_dq = kron(I44, Z * Qc) * T44 * dZ_dq +
                     kron(Z, I44) * (kron(I44, Z) * dQc_dq + kron(Qc, I44) * dZ_dq);
        }
        return Q;
    }

/* ************************************************************************* */
    gtsam::Matrix44 ConstrainedDualQuadric::normalizedMatrix(void) const {
        gtsam::Matrix44 Q = this->matrix();
        return Q / Q(3, 3);
    }

/* ************************************************************************* */
// TODO: vectorize
    AlignedBox3 ConstrainedDualQuadric::bounds() const {
        gtsam::Matrix44 dE = this->matrix();
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
        return AlignedBox3((gtsam::Vector6() << std::min(x_min, x_max),
                std::max(x_min, x_max), std::min(y_min, y_max),
                std::max(y_min, y_max), std::min(z_min, z_max),
                std::max(z_min, z_max))
                                   .finished());
    }

/* ************************************************************************* */
    bool ConstrainedDualQuadric::isBehind(const gtsam::Pose3 &cameraPose) const {
        gtsam::Pose3 rpose = cameraPose.between(this->pose());
        if (rpose.z() < 0.0) {
            return true;
        }
        return false;
    }

/* ************************************************************************* */
    bool ConstrainedDualQuadric::contains(const gtsam::Pose3 &cameraPose) const {
        gtsam::Vector4 cameraPoint =
                (gtsam::Vector4() << cameraPose.translation(), 1.0).finished();
        double pointError =
                cameraPoint.transpose() * this->matrix().inverse() * cameraPoint;
        if (pointError <= 0.0) {
            return true;
        }
        return false;
    }

/* ************************************************************************* */
    ConstrainedDualQuadric ConstrainedDualQuadric::Retract(
            const gtsam::Vector9 &v) {
        gtsam::Pose3 pose = gtsam::Pose3::Retract(v.head<6>());
        gtsam::Vector3 radii = v.tail<3>();
        return ConstrainedDualQuadric(pose, radii);
    }

/* ************************************************************************* */
    gtsam::Vector9 ConstrainedDualQuadric::LocalCoordinates(
            const ConstrainedDualQuadric &q) {
        gtsam::Vector9 v = gtsam::Vector9::Zero();
        v.head<6>() = gtsam::Pose3::LocalCoordinates(q.pose_);
        v.tail<3>() = q.radii_;
        return v;
    }

/* ************************************************************************* */
    ConstrainedDualQuadric ConstrainedDualQuadric::retract(
            const gtsam::Vector9 &v) const {
        gtsam::Pose3 pose = pose_.retract(v.head<6>());
        gtsam::Vector3 radii = radii_ + v.tail<3>();
        return ConstrainedDualQuadric(pose, radii);
    }

/* ************************************************************************* */
    gtsam::Vector9 ConstrainedDualQuadric::localCoordinates(
            const ConstrainedDualQuadric &other) const {
        gtsam::Vector9 v = gtsam::Vector9::Zero();
        v.head<6>() = pose_.localCoordinates(other.pose_);
        v.tail<3>() = other.radii_ - radii_;
        return v;
    }

/* ************************************************************************* */
    void ConstrainedDualQuadric::print(const std::string &s) const {
        cout << s;
        cout << this->matrix() << endl;
    }

/* ************************************************************************* */
    bool ConstrainedDualQuadric::equals(const ConstrainedDualQuadric &other,
                                        double tol) const {

        return this->normalizedMatrix().isApprox(other.normalizedMatrix(), tol);
    }

/* ************************************************************************* */
    void ConstrainedDualQuadric::addToValues(gtsam::Values &v,
                                             const gtsam::Key &k) {
        v.insert(k, *this);
    }

/* ************************************************************************* */
    ConstrainedDualQuadric ConstrainedDualQuadric::getFromValues(
            const gtsam::Values &v, const gtsam::Key &k) {
        return v.at<ConstrainedDualQuadric>(k);
    }

    double ConstrainedDualQuadric::calculateIntersectionError(const ConstrainedDualQuadric& e1, const ConstrainedDualQuadric& e2)
    {
        //          AXB
        // IoU = ----------
        //          AUB
        //   AXB  =  intersection
        //   AUB  =  A+B-intersection

        // Error of IoU : 1 - IoU
        double areaA = std::abs(calculateArea(e1));
        std::cout << "areaA : " << areaA << std::endl;

        double areaB = std::abs(calculateArea(e2));
        std::cout << "areaB : " << areaB << std::endl;

        double proj_inter = calculateIntersectionOnXY(e1,e2);
        double z_inter = calculateIntersectionOnZ(e1,e2);
        std::cout << "projInter : " << proj_inter << std::endl;
        std::cout << "z_inter : " << z_inter << std::endl;

        double areaIntersection = proj_inter * z_inter;
        std::cout << "areaIntersection : " << areaIntersection << std::endl;

        double MIoU = 1 - ((areaIntersection) / (areaA + areaB - areaIntersection));
        std::cout << "MIoU : " << MIoU << std::endl;
//        std::cout << "e1 : " << e1.toMinimalVector().transpose() << std::endl;
//        std::cout << "e2 : " << e2.toMinimalVector().transpose() << std::endl;

        return MIoU;
    }

    double ConstrainedDualQuadric::calculateArea(const ConstrainedDualQuadric & e)
    {
        return e.radii()[0]*e.radii()[1]*e.radii()[2]*8;
    }

    // Calculate the intersection area after projected the external cubes of two axis-aligned ellipsoids into XY-Plane.
    double ConstrainedDualQuadric::calculateIntersectionOnXY(const ConstrainedDualQuadric& e1, const ConstrainedDualQuadric& e2)
    {
        // First, get the axis-aligned pose error
        auto pose_diff = e1.pose().inverse() * e2.pose();
        double x_center1 = 0; double y_center1 = 0;

        double x_center2 = pose_diff.translation()[0];
        double y_center2 = pose_diff.translation()[1];

        double roll,pitch,yaw;
        auto ypr = pose_diff.rotation().ypr();
        yaw = ypr[0];
        pitch = ypr[1];
        roll = ypr[2];
//        quat_to_euler_zyx(pose_diff.rotation(),roll,pitch,yaw);

        double a1 = std::abs(e1.radii()[0]);
        double b1 = std::abs(e1.radii()[1]);

        double a2 = std::abs(e2.radii()[0]);
        double b2 = std::abs(e2.radii()[1]);

        // Use polygon to calculate the intersection
        Polygon polygon1, polygon2;
        double resolution = 0.001;  // m / resolution = pixel
        polygon1.add(cv::Point(a1/resolution, b1/resolution));    // cvPoint only accepts integer, so use resolution to map meter to pixel ( 0.01 resolution means: 1pixel = 0.01m )
        polygon1.add(cv::Point(-a1/resolution, b1/resolution));
        polygon1.add(cv::Point(-a1/resolution, -b1/resolution));
        polygon1.add(cv::Point(a1/resolution, -b1/resolution));

        double c_length = sqrt(a2*a2+b2*b2);

        double init_theta = CV_PI/2.0 - atan2(a2,b2);
        Vector4d angle_plus_vec;
        angle_plus_vec << 0, atan2(a2,b2)*2, CV_PI, CV_PI+atan2(a2,b2)*2;
        for( int n=0;n<4;n++){
            double angle_plus = angle_plus_vec[n];  // rotate 90deg for four times
            double point_x = c_length * cos( init_theta - yaw + angle_plus ) + x_center2;
            double point_y = c_length * sin( init_theta - yaw + angle_plus ) + y_center2;
            polygon2.add(cv::Point(point_x/resolution, point_y/resolution));
        }

        // calculate the intersection
        Polygon interPolygon;
        intersectPolygon(polygon1, polygon2, interPolygon);

        // eliminate resolution.
        double inter_area = interPolygon.area();
        double inter_area_in_m = inter_area * resolution * resolution;

        return inter_area_in_m;
    }

    double ConstrainedDualQuadric::calculateIntersectionOnZ(const ConstrainedDualQuadric& e1, const ConstrainedDualQuadric& e2)
    {
        auto pose_diff = e1.pose().inverse() * e2.pose();
        double z1 = 0; double z2 = pose_diff.translation()[2];

        bool flag_oneBigger = false;
        if( z1 > z2 )
            flag_oneBigger = true;

        double length;
        if( flag_oneBigger )
        {
            length = (z2 + e2.radii_[2]) - (z1 - e1.radii_[2]);
        }
        else
            length = (z1 + e1.radii_[2]) - (z2 - e2.radii_[2]);

        if( length < 0 )
            length = 0;     // if they are not intersected

        return length;
    }

}  // namespace gtsam_soslam
