#include "SlowDance_Initial.h"
#include "../SlowDance.h"


void SlowDance_Initial::start(mc_control::fsm::Controller & ctl_)

{
  // Compute initial base_link pose such that the Buttock surface is on the
  // chair Top surface

  auto X_0_chairTop = ctl_.robot("chair").frame("Top").position();
  auto X_0_buttock = ctl_.robot().frame("Buttock").position();
  auto buttock_offset = config_("buttock_offset", sva::PTransformd::Identity());
  auto X_base_link_buttock = buttock_offset.inv() * X_0_buttock * ctl_.robot().posW().inv();
  //Assume X_0_chairTop == X_0_buttock, e.g the two surfaces coincinde
  // Computes the floating base pose from it
  
  auto X_0_base_link = X_base_link_buttock.inv() * X_0_chairTop;
  ctl_.robot().posW(X_0_base_link);


}


bool SlowDance_Initial::run(mc_control::fsm::Controller & ctl_)
{


  output("OK");
  return true;

}
void SlowDance_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}

EXPORT_SINGLE_STATE("SlowDance_Initial", SlowDance_Initial)
