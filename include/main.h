#ifndef MAIN_H
#define MAIN_H

#include "BaseClass.h"
#include "DummyExample.h"
#include "HandMadeData.h"
#include "SoSlam.h"
#include "Viewer.h"
#include "MapDrawer.h"

bool check_file_exists(const std::string& path);

void check_input_files_exist(const std::string& img_path, const std::string& dep_path,\
                                const std::string& xml_path, const std::string& calib_file,\
                                const std::string& odom_file);

gtsam::Pose3 get_initial_pose(const std::string& path);

void run(gtsam_soslam::DataSource &data_source, \
            gtsam_soslam::BaseAssociator &associator, \
            gtsam_soslam::BaseDetector &detector, \
            const gtsam::Pose3 &initial_pose, \
            const bool &use_3D_visualization = true, \
            const bool &optimizer_batch = false
);

#endif // MAIN_H
