#include "SoSlam.h"
#include <iostream>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/inference/Factor.h>

using namespace std;

namespace gtsam_soslam{

SoSlam::SoSlam(
    DummyData data_source,
    DummyDetector detector,
    DummyAssociator associator,
    const gtsam::Pose3& initial_pose,
    const bool& optimizer_batch
    ):
    data_source_(data_source),
    detector_(detector),
    associator_(associator),
    initial_pose_(initial_pose),
    optimizer_batch_(optimizer_batch)
{
    state_ = SoSlamState(initial_pose, optimizer_batch);
    reset();
}

void SoSlam::guess_initial_values() {
    auto& s = state_;
    std::vector<boost::shared_ptr<gtsam::NonlinearFactor>> fs(s.graph_.nrFactors());
    std::transform(fs.begin(), fs.end(), fs.begin(),
               [&](decltype(fs.begin())::value_type f) 
               { return s.graph_.at(std::distance(fs.begin(), std::find(fs.begin(), fs.end(), f))); });

    for (auto f : fs) {
        if (auto pf = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) {
            auto key = pf->keys().at(0);
            if (!s.estimates_.exists(key)) {
                s.estimates_.insert(key, pf->prior());
            }
        }
    }
    std::vector<gtsam::BetweenFactor<gtsam::Pose3>*> bfs;
    for (auto f : fs) {
        if (auto bf = dynamic_cast<gtsam::BetweenFactor<gtsam::Pose3>*>(f.get())) {
            bfs.push_back(bf);
        }
    }

    bool done = false;
    while (!done) {
        gtsam::BetweenFactor<gtsam::Pose3>* bf = nullptr;
        for (auto f : bfs) {
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
    for (auto f : bfs) {
        bool all_keys_exist = std::all_of(f->keys().begin(), f->keys().end(),
                                          [&](const gtsam::Key& key) { return s.estimates_.exists(key); });
        if (!all_keys_exist) {
            s.estimates_.insert(f->keys().at(1), gtsam::Pose3());
        }
    }
    auto _ok = [](const BoundingBoxFactor& x) { return x.objectKey(); };
    std::vector<BoundingBoxFactor*> bbs;
    for (auto f : fs) {
        if (auto bb = dynamic_cast<BoundingBoxFactor *>(f.get())) {
            bbs.push_back(bb);
        }
    }

    std::map<int, std::vector<BoundingBoxFactor*>> grouped_bbs;
    for (auto bb : bbs) {
        grouped_bbs[_ok(*bb)].push_back(bb);
    }

    for (const auto& kv : grouped_bbs) {
        std::vector<gtsam::Pose3> poses;
        std::vector<AlignedBox2> points;
        for (auto bb : kv.second) {
            poses.push_back(s.estimates_.at<gtsam::Pose3>(bb->poseKey()));
            points.push_back(bb->measurement());
        }
        utils::initialize_quadric_ray_intersection(poses, points, state_).addToValues(s.estimates_, kv.second.front()->objectKey());
    }

}


// void SoSlam::spin() {
//     while (!data_source_.done()) {
//         step();
//     }
//     if (state_.system.optimizer_batch_) {
//         guess_initial_values();
//         auto& s = state_.system;
//         s.optimizer_ = gtsam::LevenbergMarquardtOptimizer(s.graph_, s.estimates_, s.optimizer_params_);
//         s.estimates_ = s.optimizer_.optimize();

//         utils::visualize(state_);
//     }
// }


void SoSlam::step() {
    // Setup state for the current step
    auto& s = state_;

    auto p = state_.prev_step;
    
    //initialize with zero
    int new_step_index = p.i + 1;
    StepState* n;
    n = &(s.this_step);
    n->i = new_step_index;

    // Get latest data from the scene (odom, images, and detections)
    std::tie(n->odom, n->depth,n->rgb) = data_source_.next(s);

    n->detections = detector_.detect(s); // be aware to deal with the situation that detector is none

    std::tie(n->new_associated, s.associated_, s.unassociated_) = associator_.associate(s);

    // Extract some labels
    // TODO handle cases where different labels used for a single quadric???
    s.labels_.clear();
    for (const auto& d : s.associated_) {
        if (d.quadric_key != -1) {
            s.labels_[d.quadric_key] = d.label;
        }
    }

    // Add new pose to the factor graph
    if (!p.isValid()) {
        s.graph_.add(gtsam::PriorFactor<gtsam::Pose3>(n->pose_key, s.initial_pose_, s.noise_prior_));
    } else {

        gtsam::Pose3 between_pose((p.odom.inverse() * n->odom).matrix());
        // gtsam::SharedNoiseModel noiseodomPtr(new gtsam::noiseModel::Diagonal(s.noise_odom_));
        gtsam::Vector6 temp;
        temp <<  0.01, 0.01, 0.01, 0.01,0.01,0.01;
        gtsam::noiseModel::Diagonal::shared_ptr noise_odom =
        gtsam::noiseModel::Diagonal::Sigmas(temp);
        s.graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(p.pose_key, n->pose_key, between_pose, noise_odom));
    }

    // Add any newly associated detections to the factor graph
    for (const auto& d : n->new_associated) {
        if (d.quadric_key == -1) {
            std::cerr << "WARN: skipping associated detection with quadric_key == -1" << std::endl;
            continue;
        }
        boost::shared_ptr<gtsam::Cal3_S2> calibPtr(new gtsam::Cal3_S2(s.calib_rgb_));
        // gtsam::SharedNoiseModel noiseboxPtr(new gtsam::Matrix(s.noise_boxes_));
        gtsam::noiseModel::Diagonal::shared_ptr noise_boxes =
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(3.0, 3.0, 3.0, 3.0));
        s.graph_.add(BoundingBoxFactor(AlignedBox2(d.bounds), calibPtr, d.pose_key, d.quadric_key, noise_boxes));
    }

    // Optimise if we're in iterative mode
    // if (!s.optimiser_batch_) {
    //     guess_initial_values();
    //     if (s.optimiser == nullptr) {
    //         s.optimiser = std::make_shared<s.optimiser_type>(s.optimiser_params);
    //     }
    //     try {
    //         s.optimiser->update(new_factors(s.graph, s.optimiser->getFactorsUnsafe()), new_values(s.estimates, s.optimiser->getLinearizationPoint()));
    //         s.estimates = s.optimiser->calculateEstimate();
    //     } catch (const std::runtime_error& e) {
    //         // Handle the exception if necessary
    //     }
    //     if (on_new_estimate) {
    //         on_new_estimate(state);
    //     }
    // }

    s.prev_step = *n;
}



void SoSlam::reset() {

    data_source_.restart();
    auto& s = state_;
    s.associated_.clear();
    s.unassociated_.clear();
    s.labels_.clear();
    s.graph_ = gtsam::NonlinearFactorGraph();
    s.estimates_ = gtsam::Values();
    s.optimizer_params_ = gtsam::ISAM2Params();
    s.optimizer_ = gtsam::ISAM2();
    // s.calib_depth_ = data_source_.calib_depth();
    s.calib_rgb_ = data_source_.calib_rgb();
    StepState new_step;
    state_.prev_step = new_step;
    state_.this_step = new_step;
}
}