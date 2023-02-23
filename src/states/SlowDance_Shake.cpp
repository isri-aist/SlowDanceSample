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
  
  //value[0].push_back({{0.0}, {0.33}, {-0.33}, {0.66}, {-0.66}, {0.0}, {0.33}, {-0.33}, {0.66}, {-0.66}});

  //neckTask->selectActiveJoints(ctl_.solver(), {"NECK_Y"}); 
  value = 0.0;

}


bool SlowDance_Shake::run(mc_control::fsm::Controller & ctl_)
{

  //auto neck_posture = ctl_.robot().mbc().q; //prend les valeurs actuelles des joints 
  
  std::vector<double> neck_posture = ctl_.robot().q()[jointIndex]; // val du NECK_Y ?
  mc_rtc::log::info("size = {}", neck_posture.size()); // size 1 et valeur 0.0
  value = 0.0;
  t_++;
  iter_++;
  value = 30;
  mc_rtc::log::info("index val. = {}", value);

  neck_posture[0]+= value;
 
  mc_rtc::log::info("index val. = {}", neck_posture[0]);
  //neckTask->target({{ jointName, neck_posture}});
    //neckTask->posture(neck_posture[0]);
   ctl_.robot().q()[jointIndex] = neck_posture;

//   for (int i = 0; i < 10; i++)
// {
//         neck_posture[0]+= 0.30;
//         
//         mc_rtc::log::info("index val. = {}", neck_posture[0]);  
// }

  
  
  // for (int a= 0; a< neck_posture.size(); a++){
  // neck_posture[jointIndex][0] = value;
  // std::for_each(value.begin(), value.end(), [](double& d) { d+=0.33;});
  
  // mc_rtc::log::info("value = {}", neck_posture[jointIndex][0]);
 
  // }


// for (int a= 0; a< 10; a++){ //10 was before neck_posture.size()
  
//  value+= 0.33;
  
// // ref postureTask->target({{ jointName, robot().qu()[jointIndex] }});
  
// mc_rtc::log::info("value = {}", value);
// //neckTask->target({{ jointName, {0.1*sin(value)} }});
// } 


//std::for_each(value[0].begin(), value[0].end(), [](double& d) { d+=0.33;});


  
  
  output("OK");
  return true;
  
  }


void SlowDance_Shake::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}


EXPORT_SINGLE_STATE("SlowDance_Shake", SlowDance_Shake)
