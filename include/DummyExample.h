#pragma once
#include "Constants.h"
#include "QuadricCamera.h"
#include "SystemState.h"

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
        return static_cast<std::vector<gtsam::Pose3>::size_type>(i)  == POSES.size();
    }

    std::tuple<gtsam::Pose3, gtsam::Matrix3 , gtsam::Vector3> next(SoSlamState& state) 
    {
        ++i;
        gtsam::Matrix3 mat = gtsam::Matrix3::Zero();
        gtsam::Vector3 vec = gtsam::Vector3::Zero();
        return std::make_tuple(gtsam::Pose3(POSES[i - 1]), mat, vec);
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
//            gtsam::Symbol symbol('q', iq);
            boost::shared_ptr<gtsam::Cal3_S2> calibPtr(new gtsam::Cal3_S2(state.calib_rgb_));
            gtsam::Vector4 bounds = QuadricCamera::project(q, POSES[current_i], calibPtr).bounds().vector();
            std::string label;
            switch (iq) {
                case 0:
                    label = "q0";
                    break;
                case 1:
                    label = "q1";
                    break;
                default:
                    label = "q0";
                    break;}
            detections.emplace_back(
                label, bounds, state.this_step.pose_key
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

    std::tuple<std::vector<Detection>, std::vector<Detection>, std::vector<Detection>> associate(SoSlamState& state)
    {
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
                d.quadric_key = gtsam::symbol(d.label[0], std::stoi(d.label.substr(1)));
                newly_associated.push_back(d);
            }
        }

        std::vector<Detection> associated, unassociated;
        for (const auto& d : flat({ds, s.associated_})) {
            if (std::find(newly_associated.begin(), newly_associated.end(), d) != newly_associated.end() ||
                std::find(s.associated_.begin(), s.associated_.end(), d) != s.associated_.end()) {
                associated.push_back(d);
            } else {
                unassociated.push_back(d);
            }
        }

        return std::make_tuple(newly_associated, associated, unassociated);
    }
};
} // namespace gtsam_soslam
