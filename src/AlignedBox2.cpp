/**
 * @file AlignedBox2.cpp
 * @author Lachlan Nicholson, thanks for your great work
 * @modified by ROB530 group6
 * @Lastest modified on 19/04/2023
 */

#include "AlignedBox2.h"

using namespace std;

namespace gtsam_soslam {

/* ************************************************************************* */
    std::vector<gtsam::Vector3> AlignedBox2::lines() const {
        std::vector<gtsam::Vector3> mLines;
        mLines.push_back(gtsam::Vector3(1, 0, -tlbr_[0]));
        mLines.push_back(gtsam::Vector3(0, 1, -tlbr_[1]));
        mLines.push_back(gtsam::Vector3(1, 0, -tlbr_[2]));
        mLines.push_back(gtsam::Vector3(0, 1, -tlbr_[3]));
        return mLines;
    }

/* ************************************************************************* */
    bool AlignedBox2::contains(const gtsam::Point2 &point) const {
        if (point.x() >= xmin() && point.x() <= xmax() && point.y() >= ymin() &&
            point.y() <= ymax()) {
            return true;
        }
        return false;
    }

/* ************************************************************************* */
    bool AlignedBox2::contains(const AlignedBox2 &other) const {
        return (other.xmin() >= this->xmin() && other.xmax() <= this->xmax() &&
                other.ymin() >= this->ymin() && other.ymax() <= this->ymax());
    }

/* ************************************************************************* */
    bool AlignedBox2::intersects(const AlignedBox2 &other) const {
        if (this->contains(other) || other.contains(*this)) {
            return false;
        }
        return !(this->xmin() > other.xmax() || this->xmax() < other.xmin() ||
                 this->ymin() > other.ymax() || this->ymax() < other.ymin());
    }

/* ************************************************************************* */
    double AlignedBox2::iou(const AlignedBox2 &other) const {
        AlignedBox2 inter_box(std::max(this->xmin(), other.xmin()),
                              std::max(this->ymin(), other.ymin()),
                              std::min(this->xmax(), other.xmax()),
                              std::min(this->ymax(), other.ymax()));

        if ((inter_box.xmax() < inter_box.xmin()) ||
            (inter_box.ymax() < inter_box.ymin())) {
            return 0.0;
        }

        double inter_area = inter_box.width() * inter_box.height();
        double this_area = this->width() * this->height();
        double other_area = other.width() * other.height();

        double iou = inter_area / (this_area + other_area - inter_area);
        assert(iou >= 0.0);
        assert(iou <= 1.0);
        return iou;
    }

/* ************************************************************************* */
    void AlignedBox2::print(const std::string &s) const {
        cout << s << this->vector().transpose() << endl;
    }

/* ************************************************************************* */
    bool AlignedBox2::equals(const AlignedBox2 &other, double tol) const {
        return tlbr_.isApprox(other.tlbr_, tol);
    }

}  // namespace gtsam_soslam
