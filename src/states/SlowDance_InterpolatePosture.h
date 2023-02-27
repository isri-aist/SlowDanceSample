#pragma once

#include <mc_control/fsm/State.h>
#include <mc_trajectory/SequenceInterpolator.h>
#include <Eigen/Core>

struct SlowDance_InterpolatePosture : mc_control::fsm::State
{

  void start(mc_control::fsm::Controller & ctl) override;

  bool run(mc_control::fsm::Controller & ctl) override;

  void teardown(mc_control::fsm::Controller & ctl) override;

private:
  using PostureInterpolator = mc_trajectory::SequenceInterpolator<Eigen::VectorXd>;
  PostureInterpolator interpolator_;
  double t_{0};
};
