//
// Created by ziqihan on 4/3/23.
//

#ifndef GTSAM_SOSLAM_BASECLASS_H
#define GTSAM_SOSLAM_BASECLASS_H

#include "SystemState.h"

#include <vector>
#include <tuple>
#include <string>
#include <unordered_set>
#include <algorithm>

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <boost/shared_ptr.hpp>

namespace gtsam_soslam {

class DataSource {
    public:
        virtual void restart() = 0;

        virtual gtsam::Vector5 calib_rgb() = 0;

        virtual bool done() const = 0;

        virtual std::tuple<gtsam::Pose3, gtsam::Matrix3, cv::Mat> next(SoSlamState &state) = 0;

        virtual ~DataSource() = default;
};

class BaseAssociator {
    public:
        virtual std::tuple<std::vector<Detection>, std::vector<Detection>, std::vector<Detection>> associate(SoSlamState& state) = 0;
        virtual ~BaseAssociator() = default;
    };

class BaseDetector {
    public:
        virtual std::vector<Detection> detect(SoSlamState& state) = 0;
        virtual ~BaseDetector() = default;
};

} // namespace gtsam_soslam
#endif //GTSAM_SOSLAM_BASECLASS_H
