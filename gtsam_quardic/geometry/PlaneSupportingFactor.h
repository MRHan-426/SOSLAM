/**
 * @file PlaneSupportingFactor.h
 * @date Mar 22, 2023
 * @author Yi-Cheng, Tien-Li
 * @brief factor between Pose3 and ConstrainedDualQuadric
 */

using namespace std;

namespace gtsam_quadrics {

class PlaneSupportingFactor
 : public gtsam::NoiseModelFactor1<ConstrainedDualQuadric>
{
  public:
    enum MeasurementModel
    {
      STANDARD,
      TRUNCATED
    };
  protected:
    A
    typedef NoiseModelFactor1<ConstrainedDualQuadric> Base; ///< base class has keys and noisemodel as private members
    gtsam::Rot3 measured_;

    // default constructor
    PlaneSupportingFactor()
      : measured_(0., 0., 0., 0.), measurementModel_(STANDARD){};

    PlaneSupportingFactor(const AlignedBox2 &measured,
                          const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                          const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                          const gtsam::SharedNoiseModel &model,
                          const SystemState &state,
                          const MeasurementModel &errorType = STANDARD)
      : Base(model, poseKey, quadricKey), 
        quadricKey_(quadricKey),
        measured_(measured),
        calibration_(calibration),
        state_(state),
        measurementModel_(errorType){};


    PlaneSupportingFactor(const AlignedBox2 &measured,
                          const boost::shared_ptr<gtsam::Cal3_S2> &calibration,
                          const gtsam::Key &poseKey, const gtsam::Key &quadricKey,
                          const gtsam::SharedNoiseModel &model,
                          const SystemState &state,
                          const std::string &errorString)
      : Base(model, poseKey, quadricKey),
        measured_(measured),
        calibration_(calibration),
        state_(state),
    {
      if (errorString == "STANDARD")
      {
        measurementModel_ = STANDARD;
      }
      else if (errorString == "TRUNCATED")
      {
        measurementModel_ = TRUNCATED;
      }
      else
      {
        throw std::logic_error(
            "The error type \"" + errorString +
            "\" is not a valid option for initializing a BoundingBoxFactor");
      }
    }

    gtsam::Matrix evaluateH1(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const;    
    gtsam::Matrix evaluateH2(
      const gtsam::Pose3 &pose, const ConstrainedDualQuadric &quadric) const;
    gtsam::Matrix evaluateH1(const gtsam::Values &x) const;
    gtsam::Matrix evaluateH2(const gtsam::Values &x) const;

    void PlaneSupportingFactor::print(const std::string& s, 
                                      const gtsam::KeyFormatter& keyFormatter) const;
};
}