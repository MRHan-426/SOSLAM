#include "SoSlam.h"

#include <iostream>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
//#include <tbb/parallel_for.h>
//#include <tbb/blocked_range.h>
//#include <tbb/global_control.h>

using namespace std;

namespace gtsam_soslam {
    SoSlam::SoSlam(
            DataSource &data_source,
            BaseAssociator &associator,
            BaseDetector &detector,
            Map* mMap,
            SoSlamState *state,
            const string &strSettingPath,
            const gtsam::Pose3 &initial_pose,
            const bool &optimizer_batch) : data_source_(data_source),
                                           associator_(associator),
                                           detector_(detector),
                                           mpMap(mMap),
                                           state_(state),
                                           initial_pose_(initial_pose),
                                           optimizer_batch_(optimizer_batch)
   {
        reset();
        cv::FileStorage fSettings(strSettingPath, cv::FileStorage::READ);
        output_quadrics_image_ = int(fSettings["output_quadrics_image"]);
        double scale = fSettings["Camera.scale"];
        auto mCalib = state_ -> calib_rgb_.K();
        mpBuilder = new Builder();
        mpBuilder->setCameraIntrinsic(mCalib, scale);
    }

    void SoSlam::guess_initial_values() {
        auto &s = *state_;
        std::vector<boost::shared_ptr<gtsam::NonlinearFactor>> fs(s.graph_.nrFactors());

        std::transform(fs.begin(), fs.end(), fs.begin(),
                       [&](decltype(fs.begin())::value_type const &f) {
                           return s.graph_.at(std::distance(fs.begin(), std::find(fs.begin(), fs.end(), f)));
                       });

        for (const auto &f: fs) {
            if (auto pf = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3> *>(f.get())) {
                auto key = pf->keys().at(0);
                if (!s.estimates_.exists(key)) {
                    s.estimates_.insert(key, pf->prior());
                }
            }
        }
        std::vector<gtsam::BetweenFactor<gtsam::Pose3> *> bfs;
        for (const auto &f: fs) {
            if (auto bf = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3> *>(f.get())) {
                bfs.push_back(bf);
            }
        }

        bool done = false;
        while (!done) {
            gtsam::BetweenFactor<gtsam::Pose3> *bf = nullptr;
            for (auto f: bfs) {
                if (s.estimates_.exists(f->keys().at(0)) && !s.estimates_.exists(f->keys().at(1))) {
                    bf = f;
                    break;
                }
            }
            if (bf == nullptr) {
                done = true;
                continue;
            }
            s.estimates_.insert(bf->keys().at(1), s.estimates_.at<gtsam::Pose3>(bf->keys().at(0)) * bf->measured());
            bfs.erase(std::remove(bfs.begin(), bfs.end(), bf), bfs.end());
        }
        for (auto f: bfs) {
            bool all_keys_exist = std::all_of(f->keys().begin(), f->keys().end(),
                                              [&](const gtsam::Key &key) { return s.estimates_.exists(key); });
            if (!all_keys_exist) {
                s.estimates_.insert(f->keys().at(1), gtsam::Pose3());
            }
        }
        // will not be used in soslam
        if (s.optimizer_batch_) {
            auto _ok_bbs = [](const BoundingBoxFactor &x) { return x.objectKey(); };

            std::vector<BoundingBoxFactor *> bbs;

            for (const auto &f: fs) {
                if (auto bb = dynamic_cast<BoundingBoxFactor *>(f.get())) {
                    bbs.push_back(bb);
                }
            }

            std::map<int, std::vector<BoundingBoxFactor *>> grouped_bbs;

            for (auto bb: bbs) {
                grouped_bbs[static_cast<int>(_ok_bbs(*bb))].push_back(bb);
            }

            for (const auto &kv: grouped_bbs) {
                std::vector<gtsam::Pose3> poses;
                std::vector<AlignedBox2> points;
                for (auto bb: kv.second) {
                    poses.push_back(s.estimates_.at<gtsam::Pose3>(bb->poseKey()));
                    points.push_back(bb->measurement());
                }
                //                s.estimates_.print();
                utils::initialize_quadric_ray_intersection(poses, points, *state_).addToValues(s.estimates_,
                                                                                              kv.second.front()->objectKey());
            }
        }
    }

    void SoSlam::spin() {
        while (!data_source_.done()) {
            step();
//            usleep(30000);
        }

        if (state_->optimizer_batch_) {
            guess_initial_values();
            auto &s = *state_;
            gtsam::LevenbergMarquardtOptimizer optimizer(s.graph_, s.estimates_, s.optimizer_params_);
            s.estimates_ = optimizer.optimize();
//            utils::visualize(s);
        } else {
            // state_.graph_.print(); // print all factors in current graph
//            utils::visualize(state_);
        }
    }

    void SoSlam::step() {
        // Define noise model
//        auto noise_prior = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Zero());
        gtsam::Vector6 temp;
        temp << 1.00, 1.00, 1.00, 1.00, 1.00, 1.00;
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
                gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(20.0));

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
        auto &s = *state_;
        auto p = state_->prev_step;
        static int count = 0;
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
        for (const auto &d: s.associated_) {
            if (d.quadric_key != 66666) {
                s.labels_[d.quadric_key] = d.label;
            }
        }

       // Add new pose to the factor graph
       if (!p.isValid()) {
           s.graph_.add(gtsam::PriorFactor<gtsam::Pose3>(n->pose_key, s.initial_pose_, noise_prior));
           guess_initial_values();
       } else {
           s.graph_.add(gtsam::PriorFactor<gtsam::Pose3>(n->pose_key, n->odom, noise_prior));
           s.estimates_.insert(n->pose_key, n->odom);
           gtsam::Pose3 between_pose((p.odom.inverse() * n->odom).matrix());
           s.graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(p.pose_key, n->pose_key, between_pose, noise_odom));
       }


        std::tuple<BoundingBoxFactor, SemanticScaleFactor, PlaneSupportingFactor, SymmetryFactor> bbs_scc_psc_syc;

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
            // point cloud process
            if(count % 20 == 0 && !n->rgb.empty()){    // RGB images are needed.
//            Eigen::VectorXd pose = mCurrFrame->cam_pose_Twc.toVector();
                mpBuilder->processFrame(n->rgb, n->depth, n->odom, 2.5); //depth thresh grab from object slam

//            mpBuilder->voxelFilter(0.01);   // Down sample threshold; smaller the finer; depend on the hardware.
                PointCloudPCL::Ptr pCloudPCL = mpBuilder->getMap();
                PointCloudPCL::Ptr pCurrentCloudPCL = mpBuilder->getCurrentMap();

                auto pCloud = utils::pclToQuadricPointCloudPtr(pCloudPCL);
                auto pCloudLocal = utils::pclToQuadricPointCloudPtr(pCurrentCloudPCL);
                mpMap->AddPointCloudList("Builder.Global Points", pCloud);
                mpMap->AddPointCloudList("Builder.Local Points", pCloudLocal);
                mpMap->addPointCloud(pCloudLocal);
            }
            if (count % 5 != 0 ) {
                s.prev_step = *n;
                count++;
                return;
            }
            count++;

            if(0) {
                //---------------------------------------------------------------------------------------------
                int blockSize = 2;
                int apertureSize = 3;
                // adjust this to get different # of keypoints
                int thresh = 180;
                double k = 0.04;
                std::vector<std::pair<double, double>> feature_points;
                cv::Mat gray_image;
                cv::cvtColor(n->rgb, gray_image, cv::COLOR_BGR2GRAY);
                cv::Mat detected_image = cv::Mat::zeros(gray_image.size(), CV_32FC1);
                cv::cornerHarris(gray_image, detected_image, blockSize, apertureSize, k);
                cv::Mat detected_norm, detected_norm_scaled;
                cv::normalize(detected_image, detected_norm, 0, 255, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
                cv::convertScaleAbs(detected_norm, detected_norm_scaled);

                for (int i = 0; i < detected_norm.rows; i++) {
                    for (int j = 0; j < detected_norm.cols; j++) {
                        if ((int) detected_norm.at<float>(i, j) > thresh) {
                            feature_points.push_back(
                                    std::make_pair((int) i, (int) j)); // x, y coordinate in matrix form
                        }
                    }
                }
            }

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
                    ConstrainedDualQuadric initial_quadric = utils::initialize_with_ssc_psc_bbs_syc(std::get<0>(bbs_scc_psc_syc), std::get<1>(bbs_scc_psc_syc), std::get<2>(bbs_scc_psc_syc), std::get<3>(bbs_scc_psc_syc), camera_pose);
                    // those factors have the same quadric key, just add once
                    initial_quadric.addToValues(s.estimates_, std::get<0>(bbs_scc_psc_syc).objectKey());
                }
            }
//            // Set up the TBB parallel_for construct
//            size_t num_threads = tbb::global_control::active_value(tbb::global_control::max_allowed_parallelism);
//            tbb::blocked_range<size_t> range(0, num_threads);
//
//            // Create a tbb::combinable container to collect optimized estimates from each thread
//            tbb::combinable<gtsam::Values> comb_estimates([&] { return s.estimates_; });
//
//            // Create the functor with the problem setup and the reference to the tbb::combinable container
//            OptimizeFunctor optimize_functor(s.graph_, comb_estimates, s.optimizer_params_);
//
//            // Run the optimization in parallel
//            tbb::parallel_for(range, optimize_functor);
//
//            // Merge the optimized estimates from each thread
//            comb_estimates.combine_each([&](const gtsam::Values& optimized_estimates) {
//                s.estimates_.update(optimized_estimates);
//            });

            gtsam::LevenbergMarquardtOptimizer optimizer(s.graph_, s.estimates_, s.optimizer_params_);
            s.estimates_ = optimizer.optimize();
            std::cout << s.graph_.error(s.estimates_) << std::endl;
//            limitFactorGraphSize(s.graph_, 100);
//            updateInitialEstimates(s.graph_, s.estimates_);

            //IOU
            static vector<vector<double>> IoUErrors(20, vector<double>(2,0)); //count, error
            //Rot
            std::vector<ConstrainedDualQuadric> groundTruthQuad = s.groundTruthes[s.dataset];
            static vector<vector<double>> angleErrors(20, vector<double>(4, 0)); //count, error y p r
            const gtsam::Values vEstimates = s.estimates_;
            auto current_ps_qs = utils::ps_and_qs_from_values(vEstimates);
            std::map<gtsam::Key, ConstrainedDualQuadric> cqs = current_ps_qs.second;
            for (auto &Obj: cqs) {
                gtsam::Key key = Obj.first;
                bool pass=true;
                auto dets = s.this_step.detections;
                for(auto &det : dets){
                    if(key == det.quadric_key)
                        pass=false;
                }
                if(pass)
                    continue;

                int idx = gtsam::Symbol(key).index();
                auto ypr = Obj.second.pose().rotation().ypr();
                auto groundTruthYpr = groundTruthQuad[idx-1].pose().rotation().ypr();

                angleErrors[idx - 1][0]++;
                for(int i =1; i<4; i++){
                    double angleError=fabs(ypr[i-1]-groundTruthYpr[i-1])*180.0/M_PI;
                    double smallestAngleError = angleError <= 90.0 ? angleError : 180.0 - angleError;
                    if(angleErrors[idx - 1][0]==1){ //init angleErrors
                        angleErrors[idx - 1][i] = smallestAngleError;
                    }
                    else{
                        int cur_count = angleErrors[idx - 1][0];
                        angleErrors[idx - 1][i] = angleErrors[idx - 1][i] * (cur_count - 1) / cur_count + smallestAngleError / cur_count;
                    }
//                    cout << smallestAngleError << " " << angleErrors[idx - 1][i] << endl;
                }
                IoUErrors[idx - 1][0]++;
                double ioUError=Obj.second.calculateIntersectionError(groundTruthQuad[idx-1],Obj.second);
                if(IoUErrors[idx - 1][0]==1){ //init angleErrors
                    IoUErrors[idx - 1][1] = ioUError;
                }
                else{
                    int cur_count=IoUErrors[idx - 1][0];
                    IoUErrors[idx - 1][1] = IoUErrors[idx - 1][1] * (cur_count - 1) / cur_count + ioUError / cur_count;
                }
//                cout<<"average IoU "<<IoUErrors[idx - 1][1]<<endl;
            }
            for (auto &Obj: cqs) {
                gtsam::Key key = Obj.first;
                int idx = gtsam::Symbol(key).index();
                cout<<"quad "<<idx<<endl;
                cout<<"average Rot: "<<endl;
                for(int i =1; i<4; i++){
                    cout<< angleErrors[idx - 1][i]<<" " ;
                }
                cout << endl;
                cout<<"average IoU: "<<IoUErrors[idx - 1][1]<<endl;
            }
        }
        s.prev_step = *n;
        if(output_quadrics_image_)
            s.this_step.imageprepared();
    }

    void SoSlam::reset() {
        data_source_.restart();
        auto &s = *state_;
        s.associated_.clear();
        s.unassociated_.clear();
        s.labels_.clear();
        s.graph_ = gtsam::NonlinearFactorGraph();
        s.estimates_ = gtsam::Values();
        s.optimizer_params_ = gtsam::LevenbergMarquardtParams();
        // s.calib_depth_ = data_source_.calib_depth();
        s.calib_rgb_ = data_source_.calib_rgb();
        StepState new_step;
        state_->prev_step = new_step;
        state_->this_step = new_step;
    }

    // Helper function
    std::tuple<BoundingBoxFactor, SemanticScaleFactor, PlaneSupportingFactor, SymmetryFactor>\
 SoSlam::add_detection_factors(const Detection &d, \
            const gtsam::noiseModel::Robust::shared_ptr &huber_boxes, \
            const gtsam::noiseModel::Robust::shared_ptr &huber_ssc, \
            const gtsam::noiseModel::Robust::shared_ptr &huber_psc, \
            const gtsam::noiseModel::Robust::shared_ptr &huber_syc) {
        if (d.quadric_key == 66666) {
            std::cerr << "WARN: skipping associated detection with quadric_key = 66666, which means None" << std::endl;
        }
        boost::shared_ptr<gtsam::Cal3_S2> calibPtr(new gtsam::Cal3_S2(state_->calib_rgb_));
        BoundingBoxFactor bbs(AlignedBox2(d.bounds), calibPtr, d.pose_key, d.quadric_key, huber_boxes, "STANDARD");
        SemanticScaleFactor ssc(d.label, calibPtr, d.pose_key, d.quadric_key, huber_ssc);
        PlaneSupportingFactor psc(d.label, calibPtr, d.pose_key, d.quadric_key, huber_psc);
        SymmetryFactor syc(AlignedBox2(d.bounds), state_->this_step.rgb, d.label, calibPtr, d.pose_key, d.quadric_key,
                           huber_syc, state_->this_step.nearest_edge_point);
        state_->graph_.add(bbs);
       state_->graph_.add(ssc);
       state_->graph_.add(psc);
//        state_.graph_.add(syc);
        return std::make_tuple(bbs, ssc, psc, syc);
    }

     std::vector<std::vector<std::pair<double, double>>>
     SoSlam::findNearestEdge(std::vector<std::pair<double, double>> &feature_points, double max_x, double max_y) {
         std::vector<std::vector<std::pair<double, double>>> nearest = std::vector<std::vector<std::pair<double, double>>>(
                 int(max_x), std::vector<std::pair<double, double>>(max_y, {0, 0}));
         for (int i = 0; i < int(max_x); i++) {
             for (int j = 0; j < int(max_y); j++) {
                 double min = DBL_MAX;
                 std::pair<double, double> nearest_edge;
                 for (int k = 0; k < feature_points.size(); k++) {
                     double distance = pow(pow(feature_points[k].first - i, 2) + pow(feature_points[k].second - j, 2),
                                           0.5);
                     if (distance < min) {
                         nearest_edge = feature_points[k];
                         min = distance;
                     }
                 }
                 nearest[i][j] = nearest_edge;
             }
         }
         return nearest;
     }

    void SoSlam::limitFactorGraphSize(gtsam::NonlinearFactorGraph &graph, size_t maxFactors) {
        size_t numFactors = graph.size();
        if (numFactors > maxFactors) {
            size_t numFactorsToRemove = numFactors - maxFactors;
            for (size_t i = 0; i < numFactorsToRemove; ++i) {
                graph.erase(graph.begin());
            }
        }
    }

    void SoSlam::updateInitialEstimates(const gtsam::NonlinearFactorGraph &graph, gtsam::Values &initialEstimates) {
        std::set<gtsam::Key> keysInGraph;
        for (const auto &factor: graph) {
            for (const gtsam::Key &key: factor->keys()) {
                keysInGraph.insert(key);
            }
        }
        for (const auto &key_value_pair: initialEstimates) {
            if (keysInGraph.find(key_value_pair.key) == keysInGraph.end()) {
                initialEstimates.erase(key_value_pair.key);
            }
        }
    }

} // namespace gtsam_soslam