#include "SlowDance.h"






SlowDance::SlowDance(mc_rbdyn::RobotModulePtr rm, double dt, const mc_rtc::Configuration & config)
: mc_control::fsm::Controller(rm, dt, config)
{
  //selfCollisionConstraint.reset();
 //qpsolver->addConstraintSet(selfCollisionConstraint);
  /* Get the complete collision constraint set */
 // selfCollisionConstraint.addCollisions(solver(), robot().mbc()->commonSelfCollisions());


  mc_rtc::log::success("SlowDance init done ");
}

bool SlowDance::run()
{
  return mc_control::fsm::Controller::run();
}

void SlowDance::reset(const mc_control::ControllerResetData & reset_data)
{
  mc_control::fsm::Controller::reset(reset_data);
}


