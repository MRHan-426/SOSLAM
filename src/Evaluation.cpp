//
// Created by ziqihan on 4/5/23.
//

// IOU
// The evaluation for shape used the Jaccard distance (1 - Intersection over Union)
// between the 3D axis-aligned bounding box of the estimated ellipsoid and the real
// bounding box after they are translated to the origin point.
// https://arxiv.org/pdf/2004.05303.pdf

#include "Evaluation.h"

namespace gtsam_soslam {
    namespace evaluate {

        // Function to calculate 2D IOU
        std::vector<double> iou_evaluation(SoSlamState &state) {
            auto pose_values = utils::ps_and_qs_from_values(state.estimates_).first;
            auto quadric_values = utils::ps_and_qs_from_values(state.estimates_).second;
            // use max posekey (latest pose)
            gtsam::Key max_key = 0;
            gtsam::Pose3 camera_pose;
            for (const auto &kv: pose_values) {
                if (kv.first > max_key) {
                    max_key = kv.first;
                    camera_pose = kv.second;
                }
            }
            boost::shared_ptr<gtsam::Cal3_S2> calibration(new gtsam::Cal3_S2(state.calib_rgb_));
            DualConic dualConic;
            AlignedBox2 result_bounds;
            std::vector<double> ious;

            for (const auto &quadric_value: quadric_values) {
                gtsam::Key quadric_key = quadric_value.first;
                ConstrainedDualQuadric quadric = quadric_value.second;
                auto detection = std::find_if(state.this_step.detections.begin(), state.this_step.detections.end(),
                                              [quadric_key](const Detection &d) {
                                                  return d.quadric_key == quadric_key;
                                              });
                dualConic = QuadricCamera::project(quadric, camera_pose, calibration);
                // check dual conic is valid for error function
                if (!dualConic.isEllipse()) {
                    throw QuadricProjectionException("Projected Conic is non-ellipse");
                }
                result_bounds = dualConic.smartBounds(calibration);

                ious.push_back(utils::iou(result_bounds, AlignedBox2(detection->bounds)));
            }
        }

        // Function to initialize QUADRICS
        std::vector<gtsam::Vector3> rot_evaluation(SoSlamState &state) {
            //TODO
            std::vector<gtsam::Vector3> temp;
            return temp;
        }
    } //namespace evaluate
} //namespace gtsam_soslam
