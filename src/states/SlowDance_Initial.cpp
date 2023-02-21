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
   

  
     
  // jointIndex = ctl_.robot().jointIndexByName(jointName);
 

  // postureTask = ctl_.getPostureTask(ctl_.robot().name()); 
  // postureTask->stiffness(200);
  

}


bool SlowDance_Initial::run(mc_control::fsm::Controller & ctl_)
{





//  auto neck_posture = ctl_.robot().mbc().q;

//   neck_posture[jointIndex][0] = 0;
//   neck_posture[jointIndex][0] = 0.33;
//   neck_posture[jointIndex][0] = 0.43;
//   neck_posture[jointIndex][0] = -0.33;

//   double b= 0.0;
//   for (int a= 0; a<  neck_posture.size(); a++){
//     neck_posture[jointIndex][0] = 2 * sin (b); //b = 0 to 0.4
  
//     b += 0.1;
   
//     mc_rtc::log::info("value = {}", neck_posture[jointIndex][index_]);
   
  // }


  // output("OK");
  // return true;
}
void SlowDance_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}

EXPORT_SINGLE_STATE("SlowDance_Initial", SlowDance_Initial)
