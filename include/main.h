#ifndef MAIN_H
#define MAIN_H

#include "BaseClass.h"
#include "DummyExample.h"
#include "SoSlam.h"

void run(gtsam_soslam::DataSource& data_source,gtsam_soslam::BaseAssociator& associator,gtsam_soslam::BaseDetector& detector);


#endif // MAIN_H
