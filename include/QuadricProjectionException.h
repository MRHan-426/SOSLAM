/**
 * @file QuadricProjectionException.h
 * @author Lachlan Nicholson, thanks for your great work
 * @unmodified
 * @Lastest review on 19/04/2023
 */

#pragma once

#include <gtsam/base/ThreadsafeException.h>
#include <gtsam/inference/Key.h>
#include <string>

namespace gtsam_soslam {

/**
 * @class QuadricProjectionException
 * Exception thrown when attemption to calculate quadric bounding box fails
 */
    class QuadricProjectionException
            : public gtsam::ThreadsafeException<QuadricProjectionException> {
    public:
        QuadricProjectionException()
                : QuadricProjectionException(std::numeric_limits<gtsam::Key>::max()) {}

        QuadricProjectionException(gtsam::Key j)
                : ThreadsafeException<QuadricProjectionException>(
                "QuadricProjectionException"),
                  j_(j) {}

        QuadricProjectionException(const std::string &description)
                : ThreadsafeException<QuadricProjectionException>(description) {}

        gtsam::Key nearbyVariable() const { return j_; }

    private:
        gtsam::Key j_;
    };

}  // namespace gtsam_soslam
