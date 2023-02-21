#include "SlowDance_Shake.h"
#include "../SlowDance.h"
#include <mc_tasks/MetaTaskLoader.h>

void SlowDance_Shake::start(mc_control::fsm::Controller & ctl_)

{
  
  
     
  jointIndex = ctl_.robot().jointIndexByName(jointName);
 

  postureTask = ctl_.getPostureTask(ctl_.robot().name()); 
  postureTask->stiffness(200);
  

}


bool SlowDance_Shake::run(mc_control::fsm::Controller & ctl_)
{


 auto neck_posture = ctl_.robot().mbc().q;

  neck_posture[jointIndex][0] = 0;
  neck_posture[jointIndex][0] = 0.33;
  neck_posture[jointIndex][0] = 0.43;
  neck_posture[jointIndex][0] = -0.33;

  double b= 0.0;
  for (int a= 0; a< neck_posture.size(); a++){
    neck_posture[jointIndex][0] = 2 * sin (b); //b = 0 to 0.4
  
    b += 0.1;
   
    mc_rtc::log::info("value = {}", neck_posture[jointIndex][index_]);
   
  }


  output("OK");
  return true;
}
void SlowDance_Shake::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}


EXPORT_SINGLE_STATE("SlowDance_Shake", SlowDance_Shake)
