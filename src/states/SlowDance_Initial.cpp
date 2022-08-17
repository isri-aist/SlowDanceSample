#include "SlowDance_Initial.h"

#include "../SlowDance.h"

void SlowDance_Initial::configure(const mc_rtc::Configuration & config)
{
}

void SlowDance_Initial::start(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}

bool SlowDance_Initial::run(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
  output("OK");
  return true;
}

void SlowDance_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}

EXPORT_SINGLE_STATE("SlowDance_Initial", SlowDance_Initial)
