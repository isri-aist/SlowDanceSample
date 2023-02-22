#include "SlowDance_Shake.h"
#include "../SlowDance.h"
#include <mc_tasks/MetaTaskLoader.h>
#include <mc_solver/ConstraintSetLoader.h>

void SlowDance_Shake::start(mc_control::fsm::Controller & ctl_)

{
  
  
     
  jointIndex = ctl_.robot().jointIndexByName(jointName);
   
  ctl_.solver().addTask(neckTask);


  postureTask = ctl_.getPostureTask(ctl_.robot().name()); 
  postureTask->stiffness(200);
  
  neckTask = std::make_shared<mc_tasks::PostureTask>(ctl_.solver(), ctl_.robot().robotIndex(), 100, 10);
  neckTask->selectActiveJoints(ctl_.solver(), {"NECK_Y"}); 

}


bool SlowDance_Shake::run(mc_control::fsm::Controller & ctl_)
{

 
  auto neck_posture = ctl_.robot().mbc().q;
 
  double value = 0.0;

  for (int a= 0; a< neck_posture.size(); a++){
  neck_posture[jointIndex][0] = sin(2*M_PI*value);
  value += 0.33;
  mc_rtc::log::info("value = {}", neck_posture[jointIndex][0]);
  }

  neckTask->posture(neck_posture); 

  
  
  output("OK");
  return true;
  
  }


void SlowDance_Shake::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}


EXPORT_SINGLE_STATE("SlowDance_Shake", SlowDance_Shake)
