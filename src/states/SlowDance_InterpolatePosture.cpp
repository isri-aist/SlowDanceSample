#include "SlowDance_InterpolatePosture.h"

#include "../SlowDance.h"

void SlowDance_InterpolatePosture::start(mc_control::fsm::Controller & ctl)
{
  // Parse the desired posture sequence.
  // We expect a vector of timed posture, where each posture is a map of
  // actuated joint name and its desired value. Some actuated joints may be
  // ommited, in which case their previous value is used.
  // Example in yaml:
  //   posture_sequence:
  //     -
  //       - 0.0
  //       - R_SHOULDER_R: -1.5
  //         L_SHOULDER_R: 1.5
  //     -
  //       - 2.0
  //       - R_SHOULDER_Y: -1.5
  //         L_SHOULDER_Y: 1.5
  //     -
  //       - 4.0
  //       - R_SHOULDER_P: -2.56
  //         R_ELBOW_P: 0.0
  //         L_SHOULDER_P: -2.56
  //         L_ELBOW_P: 0.0
  std::vector<std::pair<double, std::map<std::string, double>>> postureSequence = config_("posture_sequence");
  // Get the list of actuated joints
  const auto & rjo = ctl.robot().refJointOrder();
  // Create a vector used to store the desired value for each actuated joint
  Eigen::VectorXd desiredPosture(rjo.size());

  // Create the interpolator values
  PostureInterpolator::TimedValueVector interpolatorValues;
  // For each timed posture in the sequence
  for(const auto & [t, postureMap] : postureSequence)
  {
    // For each actuated joint
    for(int i = 0; i < rjo.size(); ++i)
    {
      const auto & actuatedJoint = rjo[i];
      // Check if we have a desired posture in the configuration
      if(postureMap.count(actuatedJoint))
      {
        // If so, put the desired joint value for this actuated joint
        desiredPosture(i) = postureMap.at(actuatedJoint);
      }
      else
      {
        // Otherwise use the current joint value
        desiredPosture(i) = ctl.robot().mbc().q[ctl.robot().jointIndexInMBC(i)][0];
      }
    }
    // Add the current posture to the interpolator values
    interpolatorValues.emplace_back(t, desiredPosture);
  }
  // Put all desired postures in the interpolator
  interpolator_.values(interpolatorValues);

  // Load the configuration for the posture task.
  // See https://jrl-umi3218.github.io/mc_rtc/json.html#MetaTask/PostureTask for
  // supported values
  // Example in yaml:
  //   posture_task:
  //     stiffness: 100
  auto & postureTask = *ctl.getPostureTask(ctl.robot().name());
  postureTask.load(ctl.solver(), config_("posture_task", mc_rtc::Configuration{}));
  output("OK");
}

bool SlowDance_InterpolatePosture::run(mc_control::fsm::Controller & ctl_)
{
  // Compute the interpolated posture at the current time
  auto desiredPosture = interpolator_.compute(t_);

  // Get the posture task
  auto & postureTask = *ctl_.getPostureTask(ctl_.robot().name());
  // Copy the current posture target
  auto posture = postureTask.posture();

  const auto & rjo = ctl_.robot().refJointOrder();
  // For each actuated joint
  for(int i = 0; i < rjo.size(); ++i)
  {
    const auto & actuatedJoint = rjo[i];
    // Set the posture target for this actuated joint to its interpolated value
    posture[ctl_.robot().jointIndexInMBC(i)][0] = desiredPosture[i];
  }
  // Change the posture target in the posture task
  postureTask.posture(posture);

  t_ += ctl_.timeStep;
  return t_ >= interpolator_.values().back().first;
}

void SlowDance_InterpolatePosture::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}

EXPORT_SINGLE_STATE("SlowDance_InterpolatePosture", SlowDance_InterpolatePosture)
