#ifndef MAIN_H
#define MAIN_H

#include "BaseClass.h"
#include "DummyExample.h"
#include "HandMadeData.h"
#include "SoSlam.h"
#include "Viewer.h"
#include "MapDrawer.h"

void run(gtsam_soslam::DataSource& data_source,\
            gtsam_soslam::BaseAssociator& associator,\
            gtsam_soslam::BaseDetector& detector,\
            const gtsam::Pose3& initial_pose
            );

#endif // MAIN_H
