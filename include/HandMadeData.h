//
// Created by ziqihan on 4/3/23.
//

#ifndef GTSAM_SOSLAM_HANDMADEDATA_H
#define GTSAM_SOSLAM_HANDMADEDATA_H
#include "Constants.h"
#include "QuadricCamera.h"
#include "SystemState.h"
#include "BaseClass.h"

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <optional>
#include <unordered_set>
#include <dirent.h>
#include <sys/stat.h>

#include "pugixml.hpp"
#include <Eigen/Dense>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/geometry/Cal3_S2.h>
#include <opencv4/opencv2/opencv.hpp> // 4.7.0

namespace gtsam_soslam {
class HandMadeData : public DataSource
{
public:
    std::string img_path_;
    std::string xml_path_;
    std::string calib_file_;
    std::vector<std::string> odom_data_;
    int num_img_;

    void restart() override {
        i = 0;
    }
    explicit HandMadeData(const std::string& img_path = "../input/img/", const std::string& xml_path = "../input/xml/", const std::string& calib_file= "../input/calib_params.txt", const std::string& odom_file= "../input/odom.txt");

    int file_num(const std::string& folder_path);

    gtsam::Vector5 calib_rgb() override;

    bool done() const override {
        return i == num_img_;
    }

    std::tuple<gtsam::Pose3, gtsam::Matrix3 , cv::Mat> next(SoSlamState& state) override;

private:
    int i;
};


class HandMadeDetector : public BaseDetector
{
public:
    std::string xml_path_;

    explicit HandMadeDetector(const std::string& xml_path = "../input/xml/") : xml_path_(xml_path), i(0){}

    std::vector<Detection> detect(SoSlamState& state) override;

private:
    int i;
};


class HandMadeAssociator : public BaseAssociator
{
public:
    HandMadeAssociator() = default;

    static std::vector<Detection> flat(const std::vector<std::vector<Detection>>& list_of_lists);

    std::tuple<std::vector<Detection>, std::vector<Detection>, std::vector<Detection>> associate(SoSlamState& state) override;

};
} // namespace gtsam_soslam

#endif //GTSAM_SOSLAM_HANDMADEDATA_H
