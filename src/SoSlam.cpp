#include "SoSlam.h"
#include <iostream>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

using namespace std;

namespace gtsam_soslam{

SoSlam::SoSlam(
    DummyData data_source,
    DummyAssociator associator,
    DummyDetector detector,
    const gtsam::Pose3& initial_pose,
    const bool& optimizer_batch
    ):
    data_source_(std::move(data_source)),
    associator_(associator),
    detector_(std::move(detector)),
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
                   [&](decltype(fs.begin())::value_type const& f)
                   { return s.graph_.at(std::distance(fs.begin(), std::find(fs.begin(), fs.end(), f))); });

    for (const auto& f : fs) {
        if (auto pf = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) {
            auto key = pf->keys().at(0);
            if (!s.estimates_.exists(key)) {
                s.estimates_.insert(key, pf->prior());
            }
        }
    }
    std::vector<gtsam::BetweenFactor<gtsam::Pose3>*> bfs;
    for (const auto& f : fs) {
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
    auto _ok_bbs = [](const BoundingBoxFactor& x) { return x.objectKey(); };
//    auto _ok_ssc = [](const SemanticScaleFactor& x) { return x.objectKey(); };

    std::vector<BoundingBoxFactor*> bbs;
//    std::vector<SemanticScaleFactor*> ssc;

    for (const auto& f : fs) {
        if (auto bb = dynamic_cast<BoundingBoxFactor *>(f.get())) {
            bbs.push_back(bb);
        }
//        else if (auto ss = dynamic_cast<SemanticScaleFactor *>(f.get())) {
//            ssc.push_back(ss);
//        }
    }

    std::map<int, std::vector<BoundingBoxFactor*>> grouped_bbs;
//    std::map<int, std::vector<SemanticScaleFactor*>> grouped_ssc;

    for (auto bb : bbs) {
        grouped_bbs[static_cast<int>(_ok_bbs(*bb))].push_back(bb);
    }
//    for (auto ss : ssc) {
//        grouped_ssc[static_cast<int>(_ok_ssc(*ss))].push_back(ss);
//    }

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

void SoSlam::spin() {
    while (!data_source_.done()) {
        step();
    }

    if (state_.optimizer_batch_) {
        guess_initial_values();
        auto& s = state_;

        // Using ISAM2 for optimization
        /* gtsam::ISAM2 isam(s.optimizer_params_);
         gtsam::ISAM2Result result = isam.update(s.graph_, s.estimates_);
         s.estimates_ = isam.calculateEstimate();*/

        // s.estimates_.print(); // print estimate values
        s.graph_.print(); // print all factors in current graph
        gtsam::LevenbergMarquardtOptimizer optimizer(s.graph_, s.estimates_, s.optimizer_params_);
        s.estimates_ = optimizer.optimize();
        utils::visualize(s);
    }
}

void SoSlam::step() {
    // Setup state for the current step
    auto& s = state_;
    auto p = state_.prev_step;

    //step index is initialized with zero
    int new_step_index = p.i + 1;
    StepState* n;
    n = &(s.this_step);
    n->i = new_step_index;
    n->pose_key = gtsam::Symbol('x',new_step_index);

    // Get latest data from the scene (odom, images, and detections)
    std::tie(n->odom, n->depth,n->rgb) = data_source_.next(s);
    n->detections = detector_.detect(s);
    std::tie(n->new_associated, s.associated_, s.unassociated_) = associator_.associate(s);

    // Extract labels
    // TODO handle cases where different labels used for a single quadric???
    for (const auto& d : s.associated_) {
        if (d.quadric_key != 66666) {
            s.labels_[d.quadric_key] = d.label;
        }
    }

    // Define noise model
    auto  noise_prior = gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector6::Zero());
    gtsam::Vector6 temp;
    temp <<  0.01, 0.01, 0.01, 0.01, 0.01, 0.01;
    gtsam::noiseModel::Diagonal::shared_ptr noise_odom = \
        gtsam::noiseModel::Diagonal::Sigmas(temp);
    gtsam::noiseModel::Diagonal::shared_ptr noise_boxes = \
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector4(3.0, 3.0, 3.0, 3.0));
    gtsam::noiseModel::Diagonal::shared_ptr noise_scc = \
        gtsam::noiseModel::Diagonal::Sigmas(gtsam::Vector1(3.0));

    // Add new pose to the factor graph
    if (!p.isValid()) {
        s.graph_.add(gtsam::PriorFactor<gtsam::Pose3>(n->pose_key, s.initial_pose_, noise_prior));
    }
    else {
        gtsam::Pose3 between_pose((p.odom.inverse() * n->odom).matrix());
        s.graph_.add(gtsam::BetweenFactor<gtsam::Pose3>(p.pose_key, n->pose_key, between_pose, noise_odom));
    }

    // Add any newly associated detections to the factor graph
    for (const auto& d : n->new_associated) {
        if (d.quadric_key == 66666) {
            std::cerr << "WARN: skipping associated detection with quadric_key = 66666, which means None" << std::endl;
            continue;
        }
        boost::shared_ptr<gtsam::Cal3_S2> calibPtr(new gtsam::Cal3_S2(s.calib_rgb_));

        s.graph_.add(BoundingBoxFactor(AlignedBox2(d.bounds), calibPtr, d.pose_key, d.quadric_key, noise_boxes));

        s.graph_.add(SemanticScaleFactor(d.label, calibPtr, d.pose_key, d.quadric_key, noise_scc));

    }

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
    s.optimizer_params_ = gtsam::LevenbergMarquardtParams();
    // s.calib_depth_ = data_source_.calib_depth();
    s.calib_rgb_ = data_source_.calib_rgb();
    StepState new_step;
    state_.prev_step = new_step;
    state_.this_step = new_step;
}
}