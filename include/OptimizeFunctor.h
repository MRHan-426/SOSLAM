#pragma once

#include <tbb/parallel_for.h>
#include <tbb/blocked_range.h>
#include <tbb/parallel_for.h>
#include <tbb/global_control.h>
#include <tbb/mutex.h>
#include <tbb/combinable.h>

#include <gtsam/nonlinear/LevenbergMarquardtParams.h>
#include <gtsam/nonlinear/ISAM2Params.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>

class OptimizeFunctor {
public:
    OptimizeFunctor(gtsam::NonlinearFactorGraph graph, tbb::combinable<gtsam::Values>& comb_estimates, gtsam::LevenbergMarquardtParams params)
            : graph_(graph), comb_estimates_(comb_estimates), params_(params) {}

    void operator()(const tbb::blocked_range<size_t>& r) const {
        for (size_t i = r.begin(); i != r.end(); ++i) {
            gtsam::LevenbergMarquardtOptimizer optimizer(graph_, comb_estimates_.local(), params_);
            gtsam::Values optimized_estimates = optimizer.optimize();

            comb_estimates_.local().update(optimized_estimates);
        }
    }

private:
    gtsam::NonlinearFactorGraph graph_;
    tbb::combinable<gtsam::Values>& comb_estimates_;
    gtsam::LevenbergMarquardtParams params_;
};

