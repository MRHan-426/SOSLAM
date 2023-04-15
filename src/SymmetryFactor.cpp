#include "QuadricProjectionException.h"
#include "SymmetryFactor.h"
#include "SemanticTable.h"
#include "QuadricCamera.h"

#include <gtsam/base/numericalDerivative.h>
#include <boost/bind/bind.hpp>
#include <Eigen/Dense>
#include <iostream>

#define NUMERICAL_DERIVATIVE true

using namespace std;

namespace gtsam_soslam
{
    gtsam::Vector SymmetryFactor::evaluateError(
        const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric,
        boost::optional<gtsam::Matrix &> H1,
        boost::optional<gtsam::Matrix &> H2) const
    {
        // In the function, the camera pose and quadratic surface inputs are first checked for validity.
        // An exception is thrown if the quadratic is behind the camera or the camera is inside the object.
        try
        {
            // check pose-quadric pair
            if (quadric.isBehind(pose))
            {
                throw QuadricProjectionException("Quadric is behind camera");
            }
            if (quadric.contains(pose))
            {
                throw QuadricProjectionException("Camera is inside quadric");
            }
            //--------------------------------------------------------------------------------------------------
            cv::Mat symmetry_image(480, 640, CV_8UC3, cv::Scalar(153, 204, 255));
            cv::Mat nearest_image(480, 640, CV_8UC3, cv::Scalar(153, 204, 255));

            for (std::pair<int, int> uniform_sample_point : uniform_sample_points_)
            {
                // draw uniform point
                int radius = 1;
                cv::Point center(uniform_sample_point.second, uniform_sample_point.first); // Note: OpenCV uses (y,x) indexing
                cv::circle(nearest_image, center, radius, cv::Scalar(255, 0, 0), 2);

                // draw new version
                cv::Point center_edge(nearest_edge_point_.at(uniform_sample_point).second, nearest_edge_point_.at(uniform_sample_point).first);
                cv::circle(nearest_image, center_edge, radius, cv::Scalar(0, 0, 255), 5);
                // cv::imshow("nearest_image", nearest_image);

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                gtsam::Pose3 quadric_pose = quadric.pose();
                gtsam::Matrix33 quadric_rotation = quadric.pose().rotation().matrix();
                gtsam::Matrix33 camera_rotation = pose.rotation().matrix();
                gtsam::Vector3 quadric_translation = quadric.pose().translation();
                gtsam::Vector3 camera_translation = pose.translation();

                gtsam::Vector3 sample_2D(uniform_sample_point.first, uniform_sample_point.second, 1);
                gtsam::Vector3 edge_2D(nearest_edge_point_.at(uniform_sample_point).first, nearest_edge_point_.at(uniform_sample_point).second, 1);
                gtsam::Vector3 symmetry_sample_2D(3);
                gtsam::Vector3 symmetry_edge_2D(3);
                gtsam::Vector4 sample_3D(4);
                gtsam::Vector4 edge_3D(4);
                gtsam::Vector4 symmetry_sample_3D;
                gtsam::Vector4 symmetry_edge_3D;

                gtsam::Matrix3 K = calibration_->K();
                static gtsam::Matrix34 I34 = gtsam::Matrix::Identity(3, 4);
                gtsam::Matrix34 extrinsic = I34 * pose.inverse().matrix();
                gtsam::Matrix43 ex_inverse = extrinsic.transpose() * (extrinsic * extrinsic.transpose()).inverse();
                gtsam::Matrix43 image2world = ex_inverse * K.inverse();
                gtsam::Matrix34 world2image = K * extrinsic;

                world2image = QuadricCamera::transformToImage(pose, calibration_);
                image2world = world2image.transpose();

                // calculate sample point in 3d space
                gtsam::Vector4 sample_ray = image2world * sample_2D;
                sample_ray.normalize();

                // Assume sample_ray is a gtsam::Vector4 representing the line in 3D space.
                // Assume box is an AlignedBox3 representing the box in 3D space.

                gtsam::Vector3 ray_dir = sample_ray.head<3>();
                gtsam::Vector3 ray_origin = camera_translation;

                gtsam::Vector3 box_min(quadric.bounds().xmin(), quadric.bounds().ymin(), quadric.bounds().zmin());
                gtsam::Vector3 box_max(quadric.bounds().xmax(), quadric.bounds().ymax(), quadric.bounds().zmax());
                // std::cout << box_min[0] << ", " << box_min[1] << ", " << box_min[2] << std::endl;

                double tmin = -std::numeric_limits<double>::infinity();
                double tmax = std::numeric_limits<double>::infinity();

                for (int i = 0; i < 3; i++)
                {
                    double t1 = (box_min(i) - ray_origin(i)) / ray_dir(i);
                    double t2 = (box_max(i) - ray_origin(i)) / ray_dir(i);
                    tmin = std::max(tmin, std::min(t1, t2));
                    tmax = std::min(tmax, std::max(t1, t2));
                }
                gtsam::Vector3 intersection_point;
                intersection_point = ray_origin + tmin * ray_dir;
                // if (tmax >= tmin)
                // {
                //     // Line intersects box, compute intersection point.
                //     intersection_point = ray_origin + tmin * ray_dir;
                // }

                sample_3D.head(3) = intersection_point;
                sample_3D[3] = 1;

                // sample_ray[3] = 0;
                // double a = sample_ray.transpose() * quadric.matrix() * sample_ray;
                // double b = 2.0 * sample_ray.transpose() * quadric.matrix() * gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                // double c = gtsam::Vector4(0.0, 0.0, 0.0, 1.0).transpose() * quadric.matrix() *
                //            gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                // double discriminant = b * b - 4.0 * a * c;
                // if (discriminant < 0.0)
                // {
                //     std::cout << "No Sample Intersection" << std::endl;
                // }
                // double t1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
                // double t2 = (-b - std::sqrt(discriminant)) / (2.0 * a);
                // gtsam::Vector4 intersection1 = sample_ray * t1;
                // gtsam::Vector4 intersection2 = sample_ray * t2;
                // if (intersection1.head<3>().dot(camera_translation) > 0)
                // {
                //     sample_3D = intersection1;
                // }
                // else
                // {
                //     sample_3D = intersection2;
                // }
                // sample_3D[3] = 1;

                // calculate edge point in 3d space
                // gtsam::Vector4 edge_ray = (image2world * edge_2D).normalize();
                // edge_ray[3] = 0;
                // a = edge_ray.transpose() * quadric.matrix() * edge_ray;
                // b = 2.0 * edge_ray.transpose() * quadric.matrix() * gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                // c = gtsam::Vector4(0.0, 0.0, 0.0, 1.0).transpose() * quadric.matrix() *
                //     gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                // discriminant = b * b - 4.0 * a * c;
                // if (discriminant < 0.0)
                // {
                //     std::cout << "No Edge Intersection" << std::endl;
                // }
                // t1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
                // t2 = (-b - std::sqrt(discriminant)) / (2.0 * a);
                // intersection1 = edge_ray * t1;
                // intersection2 = edge_ray * t2;
                // if (intersection1.head<3>().dot(camera_translation) > 0)
                // {
                //     edge_3D = intersection1;
                // }
                // else
                // {
                //     edge_3D = intersection2;
                // }
                // edge_3D[3] = 1;

                // calculate symmetry point in 3D space
                gtsam::Vector3 x_unit(1, 0, 0);
                gtsam::Vector3 x_dir = (quadric_rotation * x_unit).normalized();
                gtsam::Vector3 y_unit(0, 1, 0);
                gtsam::Vector3 y_dir = (quadric_rotation * y_unit).normalized();
                gtsam::Vector3 z_unit(0, 0, 1);
                gtsam::Vector3 z_dir = (quadric_rotation * z_unit).normalized();
                gtsam::Vector3 camera_face_dir = camera_rotation * x_unit;
                gtsam::Vector3 symmetry_plane(3);

                if (std::abs(x_dir.dot(camera_face_dir)) > std::abs(y_dir.dot(camera_face_dir)))
                    symmetry_plane = y_dir;
                else
                    symmetry_plane = x_dir;
                double d = std::pow(std::pow(quadric_translation[0], 2) + std::pow(quadric_translation[1], 2), 0.5);
                double numerator = std::abs(symmetry_plane.dot(sample_3D.head(3)) - d);
                double denominator = std::sqrt(symmetry_plane.dot(symmetry_plane));
                double distance = numerator / denominator;

                // double distance = sample_3D.head(3).dot(symmetry_plane);
                symmetry_sample_3D.head(3) = sample_3D.head(3) + 2 * distance * symmetry_plane;
                symmetry_sample_3D[3] = 1;

                // x, y, z points
                double intersection_point_x = intersection_point[0];
                double intersection_point_y = intersection_point[1];
                double intersection_point_z = intersection_point[2];
                double center_point_x = quadric_translation[0];
                double center_point_y = quadric_translation[1];
                double center_point_z = quadric_translation[2];
                double symmetry_point_x = symmetry_sample_3D[0];
                double symmetry_point_y = symmetry_sample_3D[1];
                double symmetry_point_z = symmetry_sample_3D[2];

                std::cout << "sample_3D:              " << intersection_point_x << ", " << intersection_point_y << ", " << intersection_point_z << std::endl;
                std::cout << "----------------------  " << center_point_x << ", " << center_point_y << ", " << center_point_z << std::endl;
                std::cout << "symmetry_sample_3D:     " << symmetry_point_x << ", " << symmetry_point_y << ", " << symmetry_point_z << std::endl;

                // Find the distance

                double sample_distance = std::sqrt(std::pow(center_point_x - intersection_point_x, 2) + std::pow(center_point_y - intersection_point_y, 2) + std::pow(center_point_z - intersection_point_z, 2));
                double symmetry_distance = std::sqrt(std::pow(center_point_x - symmetry_point_x, 2) + std::pow(center_point_y - symmetry_point_y, 2) + std::pow(center_point_z - symmetry_point_z, 2));
                // std::cout << "========================" << std::endl;
                // std::cout << "sample distance:        " << sample_distance << std::endl;
                // std::cout << "symmetry distance :     " << symmetry_distance << std::endl;
                // std::cout << "||||||||||||||||||||||||" << std::endl;

                // print out the symmetry point
                // cv::Mat symmetry_image(480, 640, CV_8UC3, cv::Scalar(153, 204, 255));
                // cv::Point center_sample_3D(sample_ray[1] + 100, sample_ray[0] + 100);
                // cv::circle(symmetry_image, center_sample_3D, radius, cv::Scalar(0, 255, 255), 5); // symmetry point

                // std::cout << symmetry_sample_3D[1] << ", " << symmetry_sample_3D[0] << std::endl;

                // draw 3d points in 2d plane
                cv::Point center_sample_3D(sample_ray[1] + 100, sample_ray[0] + 100);
                cv::circle(nearest_image, center_sample_3D, radius, cv::Scalar(0, 255, 255), 5);
                // cv::Point center_symmetry_sample_3D(symmetry_sample_3D[1], symmetry_sample_3D[0]);
                // cv::circle(nearest_image, center_symmetry_sample_3D, radius, cv::Scalar(255, 255, 0), 5);
                // cv::imshow("nearest_image", nearest_image);

                symmetry_sample_2D = world2image * symmetry_sample_3D;
                std::cout << symmetry_sample_2D[0] / symmetry_sample_2D[2] << ", " << symmetry_sample_2D[1] / symmetry_sample_2D[2] << std::endl;

                double symmtery_sample_2D_x =
                    (symmetry_sample_2D[0] / symmetry_sample_2D[2]) > 0 ? symmetry_sample_2D[0] /
                                                                              symmetry_sample_2D[2]
                                                                        : 0;
                double symmtery_sample_2D_y =
                    symmetry_sample_2D[1] / symmetry_sample_2D[2] > 0 ? symmetry_sample_2D[1] /
                                                                            symmetry_sample_2D[2]
                                                                      : 0;

                if ((symmtery_sample_2D_x) >= image_.rows)
                {
                    symmtery_sample_2D_x = image_.rows - 1;
                }
                if ((symmtery_sample_2D_y) >= image_.cols)
                {
                    symmtery_sample_2D_y = image_.cols - 1;
                }

                std::pair<int, int> symmetry_2D = std::make_pair((int)symmtery_sample_2D_x, (int)symmtery_sample_2D_y);
                std::cout << "symmetry_2D: " << symmetry_2D.first << ", " << symmetry_2D.second << std::endl;
                // std::pair<int, int> edge_2D_q = nearest_edge_point_.at(symmetry_2D);
                // symmetry_edge_2D << edge_2D_q.first, edge_2D_q.second, 1;

                // cv::Point center_sym_sample(symmetry_2D.second, symmetry_2D.first);
                // cv::circle(nearest_image, center_sym_sample, radius, cv::Scalar(0, 255, 255), 5);
                // cv::imshow("nearest_image", nearest_image);

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // cv::waitKey(0);
            }
            gtsam::Vector1 error(1);

            //--------------------------------------------------------------------------------------------------

            if (NUMERICAL_DERIVATIVE)
            {
                std::function<gtsam::Vector(const gtsam::Pose3 &,
                                            const ConstrainedDualQuadric &)>
                    funPtr(boost::bind(&SymmetryFactor::evaluateError, this,
                                       boost::placeholders::_1, boost::placeholders::_2,
                                       boost::none, boost::none));
                if (H1)
                {
                    Eigen::Matrix<double, 1, 6> db_dx_ =
                        gtsam::numericalDerivative21(funPtr, pose, quadric, 1e-6);
                    *H1 = db_dx_;
                }
                if (H2)
                {
                    Eigen::Matrix<double, 1, 9> db_dq_ =
                        gtsam::numericalDerivative22(funPtr, pose, quadric, 1e-6);
                    *H2 = db_dq_;
                }
            }

            return error;

            // check for nans
            if (error.array().isInf().any() || error.array().isNaN().any() ||
                (H1 && (H1->array().isInf().any() || H1->array().isNaN().any())) ||
                (H2 && (H2->array().isInf().any() || H2->array().isNaN().any())))
            {
                throw std::runtime_error("nan/inf error in bbf");
            }
        }
        // handle projection failures
        catch (QuadricProjectionException &e)
        {
            gtsam::Vector1 error = gtsam::Vector1::Ones() * 1000.0;
            if (H1)
            {
                *H1 = gtsam::Matrix::Zero(1, 6);
            }
            if (H2)
            {
                *H2 = gtsam::Matrix::Zero(1, 9);
            }
            return error;
        }
        // Just to avoid warning
        return gtsam::Vector1::Ones() * 1000.0;
    }

    /* ************************************************************************* */
    // the derivative of the error wrt camera pose (1x6)
    gtsam::Matrix SymmetryFactor::evaluateH1(
        const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
    {
        gtsam::Matrix H1;
        this->evaluateError(pose, quadric, H1, boost::none);
        return H1;
    }

    /* ************************************************************************* */
    // the derivative of the error wrt quadric (1x9)
    gtsam::Matrix SymmetryFactor::evaluateH2(
        const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const
    {
        gtsam::Matrix H2;
        this->evaluateError(pose, quadric, boost::none, H2);
        return H2;
    }

    /* ************************************************************************* */
    /** Evaluates the derivative of the error wrt pose */
    gtsam::Matrix SymmetryFactor::evaluateH1(const gtsam::Values &x) const
    {
        const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
        const ConstrainedDualQuadric quadric =
            x.at<ConstrainedDualQuadric>(this->objectKey());
        return this->evaluateH1(pose, quadric);
    }

    /* ************************************************************************* */
    /** Evaluates the derivative of the error wrt quadric */
    gtsam::Matrix SymmetryFactor::evaluateH2(const gtsam::Values &x) const
    {
        const gtsam::Pose3 pose = x.at<gtsam::Pose3>(this->poseKey());
        const ConstrainedDualQuadric quadric =
            x.at<ConstrainedDualQuadric>(this->objectKey());
        return this->evaluateH2(pose, quadric);
    }

    /* ************************************************************************* */
    // Print function
    void SymmetryFactor::print(const std::string &s,
                               const gtsam::KeyFormatter &keyFormatter) const
    {
        cout << s << "SymmetryFactor(" << keyFormatter(key1()) << ","
             << keyFormatter(key2()) << ")" << endl;
        cout << "    Label: " << label_ << endl;
        cout << "    NoiseModel: ";
        noiseModel()->print();
        cout << endl;
    }

    /* ************************************************************************* */
    // Judge function
    bool SymmetryFactor::equals(const SymmetryFactor &other,
                                double tol) const
    {
        bool equal = label_ == other.label_ &&
                     calibration_->equals(*other.calibration_, tol) &&
                     noiseModel()->equals(*other.noiseModel(), tol) &&
                     key1() == other.key1() && key2() == other.key2();
        return equal;
    }

}
