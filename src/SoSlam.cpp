#include "SoSlam.h"
#include <iostream>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <thread>
#include <mutex>

using namespace std;

namespace gtsam_soslam
{
    SoSlam::SoSlam(
        DataSource &data_source,
        BaseAssociator &associator,
        BaseDetector &detector,
        const gtsam::Pose3 &initial_pose,
        const bool &optimizer_batch) : data_source_(data_source),
                                       associator_(associator),
                                       detector_(detector),
                                       initial_pose_(initial_pose),
                                       optimizer_batch_(optimizer_batch)
    {
        state_ = SoSlamState(initial_pose, optimizer_batch);
        reset();
    }

    void SoSlam::guess_initial_values()
    {
        auto &s = state_;
        std::vector<boost::shared_ptr<gtsam::NonlinearFactor>> fs(s.graph_.nrFactors());

        std::transform(fs.begin(), fs.end(), fs.begin(),
                       [&](decltype(fs.begin())::value_type const &f)
                       {
                           return s.graph_.at(std::distance(fs.begin(), std::find(fs.begin(), fs.end(), f)));
                       });

        for (const auto &f : fs)
        {
            if (auto pf = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3> *>(f.get()))
            {
                auto key = pf->keys().at(0);
                if (!s.estimates_.exists(key))
                {
                    s.estimates_.insert(key, pf->prior());
                }
            }
        }
        std::vector<gtsam::BetweenFactor<gtsam::Pose3> *> bfs;
        for (const auto &f : fs)
        {
            if (auto bf = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3> *>(f.get()))
            {
                bfs.push_back(bf);
            }
        }

        bool done = false;
        while (!done)
        {
            gtsam::BetweenFactor<gtsam::Pose3> *bf = nullptr;
            for (auto f : bfs)
            {
                if (s.estimates_.exists(f->keys().at(0)) && !s.estimates_.exists(f->keys().at(1)))
                {
                    bf = f;
                    break;
                }
            }
            if (bf == nullptr)
            {
                done = true;
                continue;
            }
            s.estimates_.insert(bf->keys().at(1), s.estimates_.at<gtsam::Pose3>(bf->keys().at(0)) * bf->measured());
            bfs.erase(std::remove(bfs.begin(), bfs.end(), bf), bfs.end());
        }
        for (auto f : bfs)
        {
            bool all_keys_exist = std::all_of(f->keys().begin(), f->keys().end(),
                                              [&](const gtsam::Key &key)
                                              { return s.estimates_.exists(key); });
            if (!all_keys_exist)
            {
                s.estimates_.insert(f->keys().at(1), gtsam::Pose3());
            }
        }
        // will not be used in soslam
        if (s.optimizer_batch_)
        {
            auto _ok_bbs = [](const BoundingBoxFactor &x)
            { return x.objectKey(); };

            std::vector<BoundingBoxFactor *> bbs;

            for (const auto &f : fs)
            {
                if (auto bb = dynamic_cast<BoundingBoxFactor *>(f.get()))
                {
                    bbs.push_back(bb);
                }
            }

            std::map<int, std::vector<BoundingBoxFactor *>> grouped_bbs;

            for (auto bb : bbs)
            {
                grouped_bbs[static_cast<int>(_ok_bbs(*bb))].push_back(bb);
            }

            for (const auto &kv : grouped_bbs)
            {
                std::vector<gtsam::Pose3> poses;
                std::vector<AlignedBox2> points;
                for (auto bb : kv.second)
                {
                    poses.push_back(s.estimates_.at<gtsam::Pose3>(bb->poseKey()));
                    points.push_back(bb->measurement());
                }
                //                s.estimates_.print();
                utils::initialize_quadric_ray_intersection(poses, points, state_).addToValues(s.estimates_, kv.second.front()->objectKey());
            }
        }
    }

    void SoSlam::spin()
    {

        while (!data_source_.done())
        {
            step();
        }

        if (state_.optimizer_batch_)
        {
            guess_initial_values();
            auto &s = state_;

            // s.estimates_.print(); // print estimate values
            // s.graph_.print(); // print all factors in current graph
            gtsam::LevenbergMarquardtOptimizer optimizer(s.graph_, s.estimates_, s.optimizer_params_);
            s.estimates_ = optimizer.optimize();
            utils::visualize(s);
        }
        else
        {
            // state_.graph_.print(); // print all factors in current graph
            utils::visualize(state_);
        }
    }

    void SoSlam::step()
    {
        // Define noise model
        //        auto noise_prior = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Zero());
        gtsam::Vector6 temp;
        temp << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0;
        gtsam::noiseModel::Diagonal::shared_ptr noise_prior =
            gtsam::noiseModel::Diagonal::Sigmas(0.001 * temp);
        gtsam::noiseModel::Diagonal::shared_ptr noise_odom =
            gtsam::noiseModel::Diagonal::Sigmas(temp);
        gtsam::noiseModel::Diagonal::shared_ptr noise_boxes =
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(10.0, 10.0, 10.0, 10.0));
        gtsam::noiseModel::Diagonal::shared_ptr noise_ssc =
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(1.0, 1.0));
        gtsam::noiseModel::Diagonal::shared_ptr noise_psc =
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector2(10.0, 10.0));
        gtsam::noiseModel::Diagonal::shared_ptr noise_syc =
            gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(5.0));

        // Huber kernel
        auto huber_boxes = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise_boxes);
        auto huber_ssc = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise_ssc);
        auto huber_psc = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise_psc);
        auto huber_syc = gtsam::noiseModel::Robust::Create(
            gtsam::noiseModel::mEstimator::Huber::Create(1.345), noise_syc);

        // Setup state for the current step
        auto &s = state_;
        auto p = state_.prev_step;

        // step index is initialized with zero
        int new_step_index = p.i + 1;
        StepState *n;
        n = &(s.this_step);
        n->i = new_step_index;
        n->pose_key = gtsam::Symbol('x', new_step_index);

        // Get latest data from the scene (odom, images, and detections)
        std::tie(n->odom, n->depth, n->rgb) = data_source_.next(s);
        n->detections = detector_.detect(s);
        std::tie(n->new_associated, s.associated_, s.unassociated_) = associator_.associate(s);

        // Extract labels
        for (const auto &d : s.associated_)
        {
            if (d.quadric_key != 66666)
            {
                s.labels_[d.quadric_key] = d.label;
            }
        }

        // Add new pose to the factor graph
        if (!p.isValid())
        {
            s.graph_.add(gtsam::PriorFactor<gtsam::Pose3>(n->pose_key, s.initial_pose_, noise_prior));
            guess_initial_values();
        }
        else
        {
            s.graph_.add(gtsam::PriorFactor<gtsam::Pose3>(n->pose_key, n->odom, noise_prior));
            s.estimates_.insert(n->pose_key, n->odom);
            gtsam::Pose3 between_pose((p.odom.inverse() * n->odom).matrix());
            s.graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(p.pose_key, n->pose_key, between_pose, noise_odom));
        }
        //        s.graph_.print();
        //        s.estimates_.print();
        std::tuple<BoundingBoxFactor, SemanticScaleFactor, PlaneSupportingFactor, SymmetryFactor> bbs_scc_psc_syc;
        //---------------------------------------------------------------------------------------------

        // ---------------------- Find feature points ----------------------- //
        // Bounding box details
        std::vector<gtsam_soslam::Detection> boundingBoxes = s.associated_;
        for (const auto &boundingBox : boundingBoxes)
        {
            AlignedBox2 box = (AlignedBox2)boundingBox.bounds;
            double bounding_box_x = box.xmin();
            double bounding_box_y = box.ymin();
            double bounding_box_x_max = box.xmax();
            double bounding_box_y_max = box.ymax();
            double bounding_box_width = box.width();
            double bounding_box_height = box.height();
            cv::Rect region_of_interest((int)bounding_box_x, (int)bounding_box_y,
                                        (int)bounding_box_width, (int)bounding_box_height);

            cv::Mat image = n->rgb;
            cv::Mat bounding_box_image = image(region_of_interest);

            // Gray scale the image
            cv::Mat gray_image;
            cv::cvtColor(image, gray_image, cv::COLOR_BGR2GRAY);

            // Canny edge detection
            cv::Mat detected_image = cv::Mat::zeros(gray_image.size(), CV_32FC1);
            cv::Canny(gray_image, detected_image, 220, 225);

            // Normalize and scale the image
            cv::Mat detected_norm, detected_norm_scaled;
            cv::normalize(detected_image, detected_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
            cv::convertScaleAbs(detected_norm, detected_norm_scaled);
            // ====================================================================================================== //
            // Add the featured points

            std::vector<std::pair<int, int>> feature_points;
            int thresh = 108;
            for (int i = 0; i < detected_norm.rows; i++)
            {
                for (int j = 0; j < detected_norm.cols; j++)
                {
                    if ((int)detected_norm.at<float>(i, j) > thresh &&
                        (int)i < bounding_box_y_max && (int)i > bounding_box_y &&
                        (int)j < bounding_box_x_max && (int)j > bounding_box_x)
                    {
                        feature_points.emplace_back(std::make_pair((int)i, (int)j)); // x, y coordinate in matrix form
                    }
                }
            }

            //---------------------------------------------------------------------------------------------

            // calculate the closest point
            std::map<std::pair<int, int>, std::pair<int, int>> closest_map;
            std::vector<pair<int, int>> uniform_sample_points;
            int x_step = (int)bounding_box_width / 5;
            int y_step = (int)bounding_box_height / 5;
            for (int i = (int)bounding_box_x; i <= bounding_box_x + bounding_box_width; i += x_step)
            {
                for (int j = (int)bounding_box_y; j <= bounding_box_y + bounding_box_height; j += y_step)
                {
                    uniform_sample_points.emplace_back(std::make_pair(j, i)); // transfer to matrix form
                }
            }
            closest_map = findClosest(uniform_sample_points, feature_points);
            n->nearest_edge_point = closest_map;

            cv::Mat uniform_image(480, 640, CV_8UC3, cv::Scalar(153, 204, 255));
            cv::Mat nearest_image(480, 640, CV_8UC3, cv::Scalar(153, 204, 255));

            for (const auto &uniform_sample_point : uniform_sample_points)
            {
                // draw uniform point
                int radius = 1;
                cv::Scalar color(255, 0, 0);
                cv::Point center(uniform_sample_point.second, uniform_sample_point.first); // Note: OpenCV uses (y,x) indexing
                cv::circle(nearest_image, center, radius, color, 2);

                // draw new version
                cv::Point center_edge(closest_map[uniform_sample_point].second, closest_map[uniform_sample_point].first);
                cv::circle(nearest_image, center_edge, radius, cv::Scalar(255, 0, 0), 5);
                cv::imshow("nearest_image", nearest_image);

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // s.associated_[0] gtsam::Pose3 pose = s.estimates_.at<gtsam::Pose3>(s.associated_[0].pose_key);
                // gtsam::Matrix33 camera_rotation = pose.rotation().matrix();
                // gtsam::Vector3 camera_translation = pose.translation();
                // gtsam::Vector3 sample_2D(uniform_sample_point.first, uniform_sample_point.second, 1);
                // gtsam::Vector3 edge_2D(closest_map[uniform_sample_point].first, closest_map[uniform_sample_point].second);
                // gtsam::Vector3 symmetry_sample_2D(3);
                // gtsam::Vector3 symmetry_edge_2D(3);
                // gtsam::Vector4 project_line_sample(4);
                // gtsam::Vector4 project_line_edge(4);
                // gtsam::Vector4 sample_3D = QuadricCamera::transformToImage(pose, calibration_).transpose() * sample_2D;
                // gtsam::Vector4 edge_3D = QuadricCamera::transformToImage(pose, calibration_).transpose() * edge_2D;
                // gtsam::Vector4 symmetry_sample_3D;
                // gtsam::Vector4 symmetry_edge_3D;

                // gtsam::Matrix3 K = calibration_->K();
                // static gtsam::Matrix34 I34 = gtsam::Matrix::Identity(3, 4);
                // gtsam::Matrix34 extrinsic = I34 * pose.inverse().matrix();
                // gtsam::Matrix43 ex_inverse = extrinsic.transpose() * (extrinsic * extrinsic.transpose()).inverse();
                // gtsam::Matrix43 image2world = ex_inverse * K.inverse();
                // gtsam::Matrix34 world2image = K * extrinsic;

                // project_line_sample = image2world * pose.translation();

                // gtsam::Vector4 sample_ray = image2world * sample_2D;
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

                // // sample_3D = image2world * sample_2D;
                // // sample_3D /= sample_3D[3];
                // sample_3D[3] = 1;

                // gtsam::Vector4 edge_ray = image2world * edge_2D;
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
                // sample_3D[3] = 1;

                // gtsam::Vector3 x_unit(1, 0, 0);
                // gtsam::Vector3 x_dir = (quadric_rotation * x_unit).normalized();
                // gtsam::Vector3 y_unit(0, 1, 0);
                // gtsam::Vector3 y_dir = (quadric_rotation * y_unit).normalized();
                // gtsam::Vector3 z_unit(0, 0, 1);
                // gtsam::Vector3 z_dir = (quadric_rotation * z_unit).normalized();
                // gtsam::Vector3 camera_face_dir = camera_rotation * x_unit;
                // gtsam::Vector3 symmetry_plane(3);

                // if (std::abs(x_dir.dot(camera_face_dir)) > std::abs(y_dir.dot(camera_face_dir)))
                //     symmetry_plane = y_dir;
                // else
                //     symmetry_plane = x_dir;

                // double distance = sample_3D.head(3).dot(symmetry_plane);
                // symmetry_sample_3D.head(3) = sample_3D.head(3) - 2 * distance * symmetry_plane;
                // symmetry_sample_3D[3] = 1;

                // symmetry_sample_2D = world2image * symmetry_sample_3D;
                // double symmtery_sample_2D_x =
                //     (symmetry_sample_2D[0] / symmetry_sample_2D[2]) > 0 ? symmetry_sample_2D[0] /
                //                                                               symmetry_sample_2D[2]
                //                                                         : 0;
                // double symmtery_sample_2D_y =
                //     symmetry_sample_2D[1] / symmetry_sample_2D[2] > 0 ? symmetry_sample_2D[1] /
                //                                                             symmetry_sample_2D[2]
                //                                                       : 0;

                // if ((symmtery_sample_2D_x) >= image_.rows)
                // {
                //     symmtery_sample_2D_x = image_.rows - 1;
                // }
                // if ((symmtery_sample_2D_y) >= image_.cols)
                // {
                //     symmtery_sample_2D_y = image_.cols - 1;
                // }

                // std::pair<double, double> edge_2D_q = nearest_edge_point_[int(symmtery_sample_2D_x)][int(
                //     symmtery_sample_2D_y)];
                // // std::pair<double, double> edge_2D_q = {1, 1};
                // symmetry_edge_2D << edge_2D_q.first, edge_2D_q.second, 1;

                // cv::Point center_sym_sample(edge_2D_q.second, edge_2D_q.first);
                // cv::circle(nearest_image, center_sym_sample, radius, cv::Scalar(0, 255, 0), 5);
                // cv::imshow("nearest_image", nearest_image);

                // !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
                // cv::waitKey(0);
            }

            // batch optimization
            if (s.optimizer_batch_)
            {
                for (const auto &d : n->new_associated)
                {
                    bbs_scc_psc_syc = add_detection_factors(d, huber_boxes, huber_ssc, huber_psc, huber_syc);
                }
            }
            // step optimization
            else
            {
                //            guess_initial_values();
                for (const auto &d : n->new_associated)
                {
                    // add factors into graph
                    bbs_scc_psc_syc = add_detection_factors(d, huber_boxes, huber_ssc, huber_psc, huber_syc);

                    // quadric initialization
                    gtsam::KeyVector keys = s.estimates_.keys();
                    auto iter = std::find(keys.begin(), keys.end(), d.quadric_key);
                    bool found = (iter != keys.end());
                    if (!found)
                    {
                        gtsam::Pose3 camera_pose = s.estimates_.at<gtsam::Pose3>(d.pose_key);
                        ConstrainedDualQuadric initial_quadric = utils::initialize_with_ssc_psc_bbs_syc(
                            std::get<0>(bbs_scc_psc_syc), std::get<1>(bbs_scc_psc_syc), std::get<2>(bbs_scc_psc_syc),
                            std::get<3>(bbs_scc_psc_syc), camera_pose);
                        // those factors have the same quadric key, just add once
                        initial_quadric.addToValues(s.estimates_, std::get<0>(bbs_scc_psc_syc).objectKey());
                    }
                }

                //            try{
                //                s.isam_optimizer_.update(
                //                        utils::new_factors(s.graph_, s.isam_optimizer_.getFactorsUnsafe()),
                //                        utils::new_values(s.estimates_,s.isam_optimizer_.getLinearizationPoint()));
                //                s.estimates_ = s.isam_optimizer_.calculateEstimate();
                //
                //            }catch (gtsam::IndeterminantLinearSystemException &e){
                //                std::cout << "Collect Data" << std::endl;
                //            }

                gtsam::LevenbergMarquardtOptimizer optimizer(s.graph_, s.estimates_, s.optimizer_params_);
                s.estimates_ = optimizer.optimize();
                //            s.graph_.print();
                //                        s.isam_optimizer_.update(
                //                                        utils::new_factors(s.graph_, s.isam_optimizer_.getFactorsUnsafe()),
                //                                        utils::new_values(s.estimates_,s.isam_optimizer_.getLinearizationPoint()));
                //                        s.estimates_ = s.isam_optimizer_.calculateEstimate();
                std::cout << s.graph_.error(s.estimates_) << std::endl;
                //            limitFactorGraphSize(s.graph_, 100);
                //            updateInitialEstimates(s.graph_, s.estimates_);
            }
            s.prev_step = *n;
            s.this_step.imageprepared();
        }
    }

    void SoSlam::reset()
    {
        data_source_.restart();
        auto &s = state_;
        s.associated_.clear();
        s.unassociated_.clear();
        s.labels_.clear();
        s.graph_ = gtsam::NonlinearFactorGraph();
        s.estimates_ = gtsam::Values();
        s.optimizer_params_ = gtsam::LevenbergMarquardtParams();
        // s.calib_depth_ = data_source_.calib_depth();
        s.calib_rgb_ = data_source_.calib_rgb();
        StepState new_step;
        state_.prev_step = new_step;
        state_.this_step = new_step;
    }

    // Helper function
    std::tuple<BoundingBoxFactor, SemanticScaleFactor, PlaneSupportingFactor, SymmetryFactor>
    SoSlam::add_detection_factors(const Detection &d,
                                  const gtsam::noiseModel::Robust::shared_ptr &huber_boxes,
                                  const gtsam::noiseModel::Robust::shared_ptr &huber_ssc,
                                  const gtsam::noiseModel::Robust::shared_ptr &huber_psc,
                                  const gtsam::noiseModel::Robust::shared_ptr &huber_syc)
    {
        if (d.quadric_key == 66666)
        {
            std::cerr << "WARN: skipping associated detection with quadric_key = 66666, which means None" << std::endl;
        }
        boost::shared_ptr<gtsam::Cal3_S2> calibPtr(new gtsam::Cal3_S2(state_.calib_rgb_));
        BoundingBoxFactor bbs(AlignedBox2(d.bounds), calibPtr, d.pose_key, d.quadric_key, huber_boxes, "STANDARD");
        SemanticScaleFactor ssc(d.label, calibPtr, d.pose_key, d.quadric_key, huber_ssc);
        PlaneSupportingFactor psc(d.label, calibPtr, d.pose_key, d.quadric_key, huber_psc);
        SymmetryFactor syc(AlignedBox2(d.bounds), state_.this_step.rgb, d.label, calibPtr, d.pose_key, d.quadric_key,
                           huber_syc, state_.this_step.nearest_edge_point);
        state_.graph_.add(bbs);
        state_.graph_.add(ssc);
        state_.graph_.add(psc);
        state_.graph_.add(syc);
        return std::make_tuple(bbs, ssc, psc, syc);
    }

    std::vector<std::vector<std::pair<double, double>>>
    SoSlam::findNearestEdge(std::vector<std::pair<double, double>> &feature_points, double max_x, double max_y)
    {
        std::vector<std::vector<std::pair<double, double>>> nearest = std::vector<std::vector<std::pair<double, double>>>(
            int(max_x), std::vector<std::pair<double, double>>(max_y, {0, 0}));
        for (int i = 0; i < int(max_x); i++)
        {
            for (int j = 0; j < int(max_y); j++)
            {
                double min = DBL_MAX;
                std::pair<double, double> nearest_edge;
                for (long unsigned int k = 0; k < feature_points.size(); k++)
                {
                    double distance = pow(pow(feature_points[k].first - i, 2) + pow(feature_points[k].second - j, 2),
                                          0.5);
                    if (distance < min)
                    {
                        nearest_edge = feature_points[k];
                        min = distance;
                    }
                }
                nearest[i][j] = nearest_edge;
            }
        }
        return nearest;
    }

    void SoSlam::limitFactorGraphSize(gtsam::NonlinearFactorGraph &graph, size_t maxFactors)
    {
        size_t numFactors = graph.size();
        if (numFactors > maxFactors)
        {
            size_t numFactorsToRemove = numFactors - maxFactors;
            for (size_t i = 0; i < numFactorsToRemove; ++i)
            {
                graph.erase(graph.begin());
            }
        }
    }

    void SoSlam::updateInitialEstimates(const gtsam::NonlinearFactorGraph &graph, gtsam::Values &initialEstimates)
    {
        std::set<gtsam::Key> keysInGraph;
        for (const auto &factor : graph)
        {
            for (const gtsam::Key &key : factor->keys())
            {
                keysInGraph.insert(key);
            }
        }
        for (const auto &key_value_pair : initialEstimates)
        {
            if (keysInGraph.find(key_value_pair.key) == keysInGraph.end())
            {
                initialEstimates.erase(key_value_pair.key);
            }
        }
    }
    // uniform_point, feature_point
    std::map<std::pair<int, int>, std::pair<int, int>> SoSlam::findClosest(std::vector<std::pair<int, int>> uniform_points,
                                                                           std::vector<std::pair<int, int>> feature_points)
    {
        // for (int i = 0; i < feature_points.size(); i++)
        // {
        //     std::cout << feature_points[i].first << ": " << feature_points[i].second << ": ";
        // }
        // std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" << std::endl;
        std::map<std::pair<int, int>, std::pair<int, int>> closest_map;
        for (const auto &uniform_point : uniform_points)
        {
            // std::cout << "uniform_point: " << uniform_point.first << ": " << uniform_point.second << std::endl;
            double min_distance = std::numeric_limits<double>::max();
            std::pair<int, int> closest_feature_point;

            for (const auto &feature_point : feature_points)
            {
                double distance = cv::norm(cv::Point2d(feature_point.first - uniform_point.first, feature_point.second - uniform_point.second));
                // double distance = std::pow(std::pow(feature_point.first - uniform_point.first, 2) + std::pow(feature_point.second - uniform_point.second, 2), 0.5);
                if (distance < min_distance)
                {
                    std::cout << distance << std::endl;
                    min_distance = distance;
                    closest_feature_point = feature_point;
                }
            }
            // std::cout << min_distance << std::endl;
            // std::cout << closest_feature_point.first << ": " << closest_feature_point.second << std::endl;

            closest_map[uniform_point] = closest_feature_point;
        }

        return closest_map;
    }
} // namespace gtsam_soslam