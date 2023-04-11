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
            // get radius and normalize quadric matrix presentation
            gtsam::Matrix44 normalized_Q = quadric.normalizedMatrix();

            // using edge detector to find edges 2D bounding box
            int blockSize = 2;
            int apertureSize = 3;
            // adjust this to get different # of keypoints
            int thresh = 180;
            double k = 0.04;

            // x, y in image frame
            double bounding_box_x = measured_.xmin();
            double bounding_box_y = measured_.ymin();
            double bounding_box_width = measured_.width();
            double bounding_box_height = measured_.height();

            gtsam::Pose3 quadric_pose = quadric.pose();
            gtsam::Matrix33 quadric_rotation = quadric.pose().rotation().matrix();
            gtsam::Vector3 quadric_translation = quadric.pose().translation();
            gtsam::Matrix33 camera_rotation = pose.rotation().matrix();
            gtsam::Vector3 camera_translation = pose.translation();

            // cv::Rect region_of_interest(bounding_box_x, bounding_box_y, bounding_box_width, bounding_box_height);
            // cv::Mat bounding_box_image = image_(region_of_interest);
            // std::vector<std::pair<double, double>> feature_points;
            // cv::Mat gray_image;
            // cv::cvtColor(image_, gray_image, cv::COLOR_BGR2GRAY);
            // cv::Mat detected_image = cv::Mat::zeros(gray_image.size(), CV_32FC1);
            // cv::cornerHarris(gray_image, detected_image, blockSize, apertureSize, k);
            // cv::Mat detected_norm, detected_norm_scaled;
            // cv::normalize(detected_image, detected_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
            // cv::convertScaleAbs(detected_norm, detected_norm_scaled);

            // for (int i = 0; i < detected_norm.rows; i++)
            // {
            //     for (int j = 0; j < detected_norm.cols; j++)
            //     {
            //         if ((int)detected_norm.at<float>(i, j) > thresh)
            //         {
            //             feature_points.push_back(std::make_pair((int)i, (int)j)); // x, y coordinate in matrix form
            //         }
            //     }
            // }

            std::vector<pair<double, double>> uniform_sample_points;
            int x_step = bounding_box_width / 5;
            int y_step = bounding_box_height / 3;
            for (int i = bounding_box_x; i <= bounding_box_x + bounding_box_width; i += x_step)
            {
                for (int j = bounding_box_y; j <= bounding_box_y + bounding_box_height; j += y_step)
                {
                    uniform_sample_points.push_back(std::make_pair(j, i)); // transfer to matrix form
                }
            }

            std::random_device random;
            std::mt19937 gen(random());
            std::uniform_int_distribution<> dis(-3, 3);

            // near-edge sampled points in 2D plane
            // std::vector<pair<double, double>> nearedge_sample_points;
            // for (const auto &[i, j] : feature_points)
            // {
            //     int x_random = i + dis(gen);
            //     int y_random = j + dis(gen);
            //     nearedge_sample_points.push_back(std::make_pair(x_random, y_random));
            // }

            gtsam::Vector1 error(1);
            /// for loop
            int count = 0;
            // std::cout << "uniform_sample_points: " << uniform_sample_points.size() << std::endl;
            for (const auto &[i, j] : uniform_sample_points)
            {
                // std::vector<pair<double, double>> feature_points_temp = feature_points;
                double min_dis = -1;
                // pair<double, double> nearest_edge_point = nearest_edge_point_[i][j];

                // for (auto &[sx, sy] : feature_points_temp)
                // {
                //     double dis_temp = std::pow((std::pow((sx - i), 2) + std::pow((sy - j), 2)), 0.5);
                //     if (min_dis == -1 || dis_temp < min_dis)
                //     {
                //         min_dis = dis_temp;
                //         nearest_edge_point = std::make_pair(sx, sy);
                //     }
                //     // std::cout << "min_dis: " << min_dis << std::endl;
                // }

                gtsam::Vector3 sample_2D(i, j, 1);
                gtsam::Vector3 edge_2D(nearest_edge_point_[i][j].first, nearest_edge_point_[i][j].second, 1);
                gtsam::Vector3 symmetry_sample_2D(3);
                gtsam::Vector3 symmetry_edge_2D(3);
                gtsam::Vector4 project_line_sample(4);
                gtsam::Vector4 project_line_edge(4);

                double point_quadric_loss;
                double sign = 1;
                gtsam::Vector4 sample_3D = QuadricCamera::transformToImage(pose, calibration_).transpose() * sample_2D;
                gtsam::Vector4 edge_3D = QuadricCamera::transformToImage(pose, calibration_).transpose() * edge_2D;
                gtsam::Vector4 symmetry_sample_3D;
                gtsam::Vector4 symmetry_edge_3D;

                gtsam::Matrix3 K = calibration_->K();
                static gtsam::Matrix34 I34 = gtsam::Matrix::Identity(3, 4);
                gtsam::Matrix34 extrinsic = I34 * pose.inverse().matrix();
                gtsam::Matrix43 ex_inverse = extrinsic.transpose() * (extrinsic * extrinsic.transpose()).inverse();
                gtsam::Matrix43 image2world = ex_inverse * K.inverse();
                gtsam::Matrix34 world2image = K * extrinsic;

                project_line_sample = image2world * pose.translation();
                // gtsam::Vector3 line_dir(3) =
                // if (project_line_sample.head(3).transpose() * quadric.pose().translation() > 0)
                // {
                //     sign = 1;
                // }
                // else
                //     sign = -1;

                gtsam::Vector4 sample_ray = image2world * sample_2D;
                sample_ray[3] = 0;
                double a = sample_ray.transpose() * quadric.matrix() * sample_ray;
                double b = 2.0 * sample_ray.transpose() * quadric.matrix() * gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                double c = gtsam::Vector4(0.0, 0.0, 0.0, 1.0).transpose() * quadric.matrix() * gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                double discriminant = b * b - 4.0 * a * c;
                if (discriminant < 0.0)
                {
                    std::cout << "No Sample Intersection" << std::endl;
                }
                double t1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
                double t2 = (-b - std::sqrt(discriminant)) / (2.0 * a);
                gtsam::Vector4 intersection1 = sample_ray * t1;
                gtsam::Vector4 intersection2 = sample_ray * t2;
                if (intersection1.head<3>().dot(camera_translation) > 0)
                {
                    sample_3D = intersection1;
                }
                else
                {
                    sample_3D = intersection2;
                }

                // sample_3D = image2world * sample_2D;
                // sample_3D /= sample_3D[3];
                sample_3D[3] = 1;
                // std::cout << "ss3d: " << sample_3D[0] << "ss3d: " << sample_3D[1] << "ss3d: " << sample_3D[2] << "ss3d: " << sample_3D[3] << std::endl;

                // while (1)
                // {
                //     std::cout << "check4" << endl;

                //     gtsam::Vector4 temp(pose.x(), pose.y(), pose.z(), 1);
                //     double camera_quadric_loss = temp.transpose() * normalized_Q * temp;
                //     temp = temp + project_line_sample * 0.5 * sign;
                //     point_quadric_loss = temp.transpose() * normalized_Q * temp;
                //     std::cout << camera_quadric_loss << endl;
                //     std::cout << point_quadric_loss << endl;
                //     // if (point_quadric_loss > camera_quadric_loss)
                //     //     // sign *= -1;
                //     //     continue;
                //     if (point_quadric_loss < 10.0)
                //     {
                //         sample_3D = temp;
                //         break;
                //     }
                // }

                gtsam::Vector4 edge_ray = image2world * edge_2D;
                edge_ray[3] = 0;
                a = edge_ray.transpose() * quadric.matrix() * edge_ray;
                b = 2.0 * edge_ray.transpose() * quadric.matrix() * gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                c = gtsam::Vector4(0.0, 0.0, 0.0, 1.0).transpose() * quadric.matrix() * gtsam::Vector4(0.0, 0.0, 0.0, 1.0);
                discriminant = b * b - 4.0 * a * c;
                if (discriminant < 0.0)
                {
                    std::cout << "No Edge Intersection" << std::endl;
                }
                t1 = (-b + std::sqrt(discriminant)) / (2.0 * a);
                t2 = (-b - std::sqrt(discriminant)) / (2.0 * a);
                intersection1 = edge_ray * t1;
                intersection2 = edge_ray * t2;
                if (intersection1.head<3>().dot(camera_translation) > 0)
                {
                    edge_3D = intersection1;
                }
                else
                {
                    edge_3D = intersection2;
                }
                sample_3D[3] = 1;

                // edge_3D = image2world * edge_2D;
                // edge_3D /= edge_3D[3];

                // while (1)
                // {
                //     std::cout << "check5" << endl;

                //     gtsam::Vector4 temp = gtsam::Vector4(pose.x(), pose.y(), pose.z(), 1);
                //     double camera_quadric_loss = temp.transpose() * normalized_Q * temp;
                //     temp = temp + project_line_edge * 0.5 * sign;
                //     point_quadric_loss = temp.transpose() * normalized_Q * temp;
                //     if (point_quadric_loss > camera_quadric_loss)
                //         sign = -1;
                //     if (point_quadric_loss < 10)
                //     {
                //         edge_3D = temp;
                //         break;
                //     }
                // }

                // Eigen::MatrixXd matrix(normalized_Q.rows(), normalized_Q.cols());
                // for (int i = 0; i < normalized_Q.rows(); i++)
                // {
                //     for (int j = 0; j < normalized_Q.cols(); j++)
                //     {
                //         matrix(i, j) = normalized_Q(i, j);
                //     }
                // }

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

                double distance = sample_3D.head(3).dot(symmetry_plane);
                symmetry_sample_3D.head(3) = sample_3D.head(3) - 2 * distance * symmetry_plane;
                symmetry_sample_3D[3] = 1;
                // std::cout << "2d: " << sample_2D[0] << " 2d: " << sample_2D[1] << " 2d: " << sample_2D[2] << std::endl;

                // std::cout << "3d: " << sample_3D[0] << " 3d: " << sample_3D[1] << " 3d: " << sample_3D[2] << " 3d: " << sample_3D[3] << std::endl;
                // std::cout << "ss3d: " << symmetry_sample_3D[0] << " ss3d: " << symmetry_sample_3D[1] << " ss3d: " << symmetry_sample_3D[2] << " ss3d: " << symmetry_sample_3D[3] << std::endl;
                // std::cout << "ss2d: " << sample_2D[0] << " ss2d: " << sample_2D[1] << " ss2d: " << sample_2D[2] << std::endl;
                // Eigen::VectorXd z_axis(4);
                // z_axis << 0, 0, 1, 1;
                // Eigen::EigenSolver<Eigen::MatrixXd> eigensolver(matrix);
                // Eigen::VectorXd eigenvalues = eigensolver.eigenvalues().real();
                // Eigen::MatrixXd eigenvectors = eigensolver.eigenvectors().real();
                // Eigen::VectorXd upward;
                // double max_value = 0;
                // int index_z;

                // for (int i = 0; i < 3; i++)
                // {

                //     Eigen::VectorXd desired_eigenvector = eigenvectors.col(i);
                //     if (std::abs(z_axis.dot(desired_eigenvector)) > max_value)
                //     {
                //         max_value = std::abs(z_axis.dot(desired_eigenvector));
                //         upward = desired_eigenvector;
                //         index_z = i;
                //     }
                // }

                // max_value = 0;
                // Eigen::VectorXd horizon;
                // for (int i = 0; i < 3; i++)
                // {
                //     if (i == index_z)
                //         continue;
                //     Eigen::VectorXd temp(4);
                //     temp << pose.x(), pose.y(), pose.z(), 1;
                //     Eigen::VectorXd desired_eigenvector = eigenvectors.col(i);
                //     if (std::abs(temp.dot(desired_eigenvector)) > max_value)
                //     {
                //         max_value = std::abs(temp.dot(desired_eigenvector));
                //         horizon = desired_eigenvector;
                //     }
                // }

                // Eigen::VectorXd sample_point(3);
                // sample_point << sample_3D[0], sample_3D[1], sample_3D[2];
                // Eigen::VectorXd quadric_pose_3d(3);
                // quadric_pose_3d << quadric.pose().x(), quadric.pose().y(), quadric.pose().z();
                // double d = -quadric_pose_3d.dot(horizon.head(3));

                // Eigen::VectorXd sample_point_sym = sample_point - 2 * (sample_point.dot(horizon.head(3)) + d) / horizon.head(3).squaredNorm() * horizon.head(3);
                // symmetry_sample_3D << sample_point_sym(0), sample_point_sym(1), sample_point_sym(2), 1;
                // std::cout << "3D sample" << sample_point_sym(0) << " " << sample_point_sym(1) << " " << sample_point_sym(2) << std::endl;

                symmetry_sample_2D = world2image * symmetry_sample_3D;
                double symmtery_sample_2D_x = (symmetry_sample_2D[0] / symmetry_sample_2D[2]) > 0 ? symmetry_sample_2D[0] / symmetry_sample_2D[2] : 0;
                double symmtery_sample_2D_y = symmetry_sample_2D[1] / symmetry_sample_2D[2] > 0 ? symmetry_sample_2D[1] / symmetry_sample_2D[2] : 0;

                if ((symmtery_sample_2D_x) >= image_.rows)
                {
                    symmtery_sample_2D_x = image_.rows - 1;
                }
                if ((symmtery_sample_2D_y) >= image_.cols)
                {
                    symmtery_sample_2D_y = image_.cols - 1;
                }

                // std::cout << "x: " << symmtery_sample_2D_x << " y: " << symmtery_sample_2D_y << std::endl;
                // min_dis = -1;
                // for (auto &[sx, sy] : feature_points_temp)
                // {
                //     double dis_temp = std::pow((std::pow((sx - symmtery_sample_2D_x), 2) + std::pow((sy - symmtery_sample_2D_y), 2)), 0.5);
                //     if (min_dis == -1 || dis_temp < min_dis)
                //     {
                //         min_dis = dis_temp;
                //         symmetry_edge_2D << sx, sy, 1;
                //     }
                // }

                std::pair<double, double> edge_2D_q = nearest_edge_point_[int(symmtery_sample_2D_x)][int(symmtery_sample_2D_y)];
                // std::pair<double, double> edge_2D_q = {1, 1};
                symmetry_edge_2D << edge_2D_q.first, edge_2D_q.second, 1;

                // std::cout << "sample_min_dis: " << min_dis << std::endl;

                symmetry_edge_3D = QuadricCamera::transformToImage(pose, calibration_).transpose() * symmetry_edge_2D;
                symmetry_edge_3D = image2world * symmetry_edge_2D;
                symmetry_edge_3D[0] /= symmetry_edge_3D[3];
                symmetry_edge_3D[1] /= symmetry_edge_3D[3];
                symmetry_edge_3D[2] /= symmetry_edge_3D[3];

                // std::cout << "3D edge" << symmetry_edge_3D(0) << " " << symmetry_edge_3D(1) << " " << symmetry_edge_3D(2) << std::endl;

                double distance1 = (sample_3D - edge_3D).norm();
                double distance2 = (symmetry_sample_3D - symmetry_edge_3D).norm();
                // std::cout << "distance1: " << distance1 << "distance2: " << distance2 << std::endl;
                error[0] += std::pow((distance1 - distance2), 2);
                // std::cout << "percentage: " << (float)count / (float)uniform_sample_points.size() << std::endl;
            }
//            error[0] /= 1000;

//            std::cout << "syc error: " << error[0] << std::endl;

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

