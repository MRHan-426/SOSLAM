#include "SoSlam.h"
#include <iostream>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>

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
    // reset();
}

// void SoSlam::guess_initial_values() {
//     SystemState& s = state_.system;
//     std::vector<gtsam::Factor::shared_ptr> fs(s.graph_.nrFactors());
//     std::transform(fs.begin(), fs.end(), fs.begin(),
//                    [&](decltype(*fs.begin()) f) { return s.graph_.at(f - fs.begin()); });
//     for (auto f : fs) {
//         if (auto pf = dynamic_cast<gtsam::PriorFactor<gtsam::Pose3>*>(f.get())) {
//             auto key = pf->keys().at(0);
//             if (!s.estimates_.exists(key)) {
//                 s.estimates_.insert(key, pf->prior());
//             }
//         }
//     }
//     std::vector<gtsam::gtsam::BetweenFactor<gtsam::Pose3>*> bfs;
//     for (auto f : fs) {
//         if (auto bf = dynamic_cast<gtsam::gtsam::BetweenFactor<gtsam::Pose3>*>(f.get())) {
//             bfs.push_back(bf);
//         }
//     }
//     bool done = false;
//     while (!done) {
//         gtsam::gtsam::BetweenFactor<gtsam::Pose3>* bf = std::nullptr;
//         for (auto f : bfs) {
//             if (s.estimates_.exists(f->keys().at(0)) && !s.estimates_.exists(f->keys().at(1))) {
//                 bf = f;
//                 break;
//             }
//         }
//         if (bf == std::nullptr) {
//             done = true;
//             continue;
//         }
//         s.estimates_.insert(bf->keys().at(1), s.estimates_.at<gtsam::Pose3>(bf->keys().at(0)) * bf->measured());
//         bfs.erase(std::remove(bfs.begin(), bfs.end(), bf), bfs.end());
//     }
//     for (auto f : bfs) {
//         bool all_keys_exist = std::all_of(f->keys().begin(), f->keys().end(),
//                                           [&](const gtsam::Key& key) { return s.estimates_.exists(key); });
//         if (!all_keys_exist) {
//             s.estimates_.insert(f->keys().at(1), gtsam::Pose3());
//         }
//     }
//     auto _ok = [](const BoundingBoxFactor& x) { return x.objectKey(); };
//     std::vector<BoundingBoxFactor*> bbs;
//     for (auto f : fs) {
//         if (auto bb = dynamic_cast<BoundingBoxFactor*>(f.get())) {
//             bbs.push_back(bb);
//         }
//     }
//     std::sort(bbs.begin(), bbs.end(), [&](auto a, auto b) { return _ok(*a) < _ok(*b); });
//     for (auto qbbs : groupby(bbs, _ok)) {
//         std::vector<gtsam::Pose3> poses;
//         std::vector<gtsam::Point3> points;
//         for (auto bb : qbbs.second) {
//             poses.push_back(s.estimates_.at<gtsam::Pose3>(bb->poseKey()));
//             points.push_back(bb->measurement());
//         }
//         utils::initialize_quadric_ray_intersection(poses, points, state_).addToValues(s.estimates_, qbbs.second.front()->objectKey());
//     }
// }


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


// void SoSlam::step()
// {
//     // Setup state for the current step
//     auto& s = state_.system;
//     auto& p = state_.prev_step;
//     auto n = StepState((p == std::nullptr) ? 0 : p->i + 1);
//     state_.this_step = n;

//     // Get latest data from the scene (odom, images, and detections)
//     std::tie(n.odom, n.rgb, n.depth) = data_source_ -> next(state_);
//     // if (visual_odometry)
//     //     n.odom = visual_odometry -> odom(state_);
//     n.detections = (detector_ == std::nullptr) ? std::vector<Detection>() : detector_->detect(state_);
//     std::tie(n.new_associated, s.associated, s.unassociated) = associator_->associate(state_);

//     // Extract some labels
//     s.labels_.clear();
//     for (const auto& d : s.associated_)
//     {
//         if (d.quadric_key != gtsam::Symbol('Q', 0))
//             s.labels_[d.quadric_key] = d.label;
//     }

//     // Add new pose to the factor graph
//     if (p == std::nullptr)
//         s.graph_.emplace_shared<gtsam::PriorFactor<gtsam::Pose3>>(n.pose_key, s.initial_pose_, s.noise_prior_);
//     else
//         s.graph_.emplace_shared<gtsam::BetweenFactor<gtsam::Pose3>>(p->pose_key, n.pose_key,
//             // TODO:Some Bugs here
//             gtsam::Pose3(((p->odom == std::nullptr) ? gtsam::Pose3() : p->odom->inverse()) *
//                          ((n.odom == std::nullptr) ? gtsam::Pose3() : n.odom)).matrix(),
//             s.noise_odom_);

//     // Add any newly associated detections to the factor graph
//     for (const auto& d : n.new_associated)
//     {
//         if (d.quadric_key == gtsam::Symbol('Q', 0))
//         {
//             std::cerr << "WARN: skipping associated detection with quadric_key == None" << std::endl;
//             continue;
//         }
//         s.graph_.emplace_shared<BoundingBoxFactor>(AlignedBox2(d.bounds), 
//             gtsam::Cal3_S2(s.calib_rgb_), d.pose_key, d.quadric_key, s.noise_boxes_);
//     }

//     // Optimise if we're in iterative mode
//     if (!s.optimizer_batch_)
//     {
//         guess_initial_values();

//         if (!s.optimizer_.has_value())
//             s.optimizer_ = std::make_shared<gtsam::LevenbergMarquardtOptimizer>(s.graph_, s.estimates_, s.optimizer_params_);
//         try
//         {
//             // gtsam::ISAM2 
//             s.optimizer_-> update(utils::new_factors(s.graph_, s.optimizer_->getFactorsUnsafe()), utils::new_values(s.estimates_, s.optimizer_->getLinearizationPoint()));
//             s.estimates_ = s.optimizer_->optimize();
//         }
//         utils::visualize(state_);
//     }
//     state_.prev_step = std::make_shared<StepState>(n);
// }


// void SoSlam::reset() {

//     data_source_.restart();
//     auto& s = state_.system;
//     s.associated_.clear();
//     s.unassociated_.clear();
//     s.labels_.clear();
//     s.graph_ = gtsam::NonlinearFactorGraph();
//     s.estimates_ = gtsam::Values();
//     s.optimizer_params_ = std::monostate{};
//     s.optimizer_ = std::monostate{};
//     s.calib_depth_ = data_source_.calib_depth();
//     s.calib_rgb_ = data_source_.calib_rgb();
//     StepState new_step;
//     state_.prev_step = new_step;
//     state_.this_step = new_step;
// }
}
