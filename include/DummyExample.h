#pragma once
#include "Constants.h"
#include "QuadricCamera.h"
#include <vector>
#include <optional>
#include <Eigen/Dense>
#include <unordered_set>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>

namespace gtsam_soslam {
class DummyData
{
public:
    void restart() {
        i = 0;
    }
    DummyData() {
        restart();
    }

    gtsam::Vector5 calib_rgb() {
        double fx = 525.0;
        double fy = 525.0;
        double s = 0.0;
        double u0 = 160.0;
        double v0 = 120.0;
        
        gtsam::Vector5 result;
        result<<fx, fy, s, u0, v0;
        return result;
    }


    bool done() const {
        return i == POSES.size();
    }

    std::tuple<std::optional<gtsam::Pose3>, std::optional<Eigen::MatrixXd>, std::optional<Eigen::MatrixXd>> next(SoSlamState& state) 
    {
        ++i;
        return std::make_tuple(std::optional<gtsam::Pose3>(gtsam::Pose3(POSES[i - 1].matrix())), std::nullopt, std::nullopt);
    }



private:
    int i;
    const std::vector<gtsam::Pose3> POSES = Constants::POSES;

};


class DummyDetector {
public:
    DummyDetector() : i(0) {}

    std::vector<Detection> detect(SoSlamState& state) {
        int current_i = i;
        ++i;

        std::vector<Detection> detections;
        for (size_t iq = 0; iq < QUADRICS.size(); ++iq) {
            const auto& q = QUADRICS[iq];
            gtsam::Symbol symbol('q', iq);
            boost::shared_ptr<gtsam::Cal3_S2> calibPtr(new gtsam::Cal3_S2(state.calib_rgb_));
            auto bounds = QuadricCamera::project(q, POSES[current_i], calibPtr).bounds().vector();

            detections.emplace_back(
                Detection{symbol.string(), bounds, state.this_step.pose_key}
            );
        }

        return detections;
    }

private:
    int i;
    const std::vector<gtsam::Pose3> POSES = Constants::POSES;
    const std::vector<ConstrainedDualQuadric> QUADRICS = Constants::QUADRICS;
};


class DummyAssociator {
public:
    static std::vector<Detection> flat(const std::vector<std::vector<Detection>>& list_of_lists) {
        std::vector<Detection> result;
        for (const auto& sublist : list_of_lists) {
            result.insert(result.end(), sublist.begin(), sublist.end());
        }
        return result;
    }

        std::tuple<std::vector<Detection>, std::vector<Detection>, std::vector<Detection>> associate(SoSlamState& state) {
            auto& s = state;
            auto& n = state.this_step;
            std::vector<Detection> new_ds = (!n.isValid()) ? std::vector<Detection>{} : n.detections;

            std::vector<Detection> ds = new_ds;
            ds.insert(ds.end(), s.unassociated_.begin(), s.unassociated_.end());

        std::unordered_set<std::string> associated_labels;
        for (const auto& d : s.associated_) {
            associated_labels.insert(d.label);
        }

        std::vector<Detection> newly_associated;
        for (auto& d : ds) {
            int count = 0;
            for (const auto& x : ds) {
                if (x.label == d.label) {
                    ++count;
                }
            }

            if (associated_labels.count(d.label) > 0 || count >= 3) {
                newly_associated.push_back(d);
                d.quadric_key = gtsam::symbol(d.label[0], std::stoi(d.label.substr(1)));
            }
        }

        std::vector<Detection> detection_intersection, detection_difference;
        for (const auto& d : flat({ds, s.associated_})) {
            if (std::find(newly_associated.begin(), newly_associated.end(), d) != newly_associated.end() ||
                std::find(s.associated_.begin(), s.associated_.end(), d) != s.associated_.end()) {
                detection_intersection.push_back(d);
            } else {
                detection_difference.push_back(d);
            }
        }

        return std::make_tuple(newly_associated, detection_intersection, detection_difference);
    }
};
} // namespace gtsam_soslam
