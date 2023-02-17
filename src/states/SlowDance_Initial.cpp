#include "SlowDance_Initial.h"
#include "../SlowDance.h"


void SlowDance_Initial::start(mc_control::fsm::Controller & ctl_)
{
  // Compute initial base_link pose such that the Buttock surface is on the
  // chair Top surface

  // auto X_0_chairTop = ctl_.robot("chair").frame("Top").position();
  // auto X_0_buttock = ctl_.robot().frame("Buttock").position();
  // auto buttock_offset = config_("buttock_offset", sva::PTransformd::Identity());
  // auto X_base_link_buttock = buttock_offset.inv() * X_0_buttock * ctl_.robot().posW().inv();
  // Assume X_0_chairTop == X_0_buttock, e.g the two surfaces coincinde
  // Computes the floating base pose from it
  // auto X_0_base_link = X_base_link_buttock.inv() * X_0_chairTop;
  // ctl_.robot().posW(X_0_base_link);
   
  //postureTask->target("NECK_Y", ctl.robot().frame("Neck").position());
 

// desired_pos_vector.push_back({
 
//   {0.0}, 
//   {-0.02}, 
//   {-1.34}, 
//   {1.4}, 
//   {0.1}, 
//   {0.13}, 
//   {0.0}, 
//   {0.02}, 
//   {-1.34}, 
//   {1.4}, 
//   {0.1}, 
//   {-0.13}, 
//   {0.5}, 
//   {0.0}, 
//   {0.0}, 
//   {0.0},
//   {-0.052}, 
//   {-0.174444}, 
//   {0.0}, 
//   {-1.44}, 
//   {0.0}, 
//   {0.0}, 
//   {0.0}, 
//   {0.0}, 
//   {0.0}, 
//   {-0.052}, 
//   {0.174444}, 
//   {0.0}, 
//   {-1.44},
//   {0.0},
//   {0.0},
//   {0.0}, 
//   {0.0}, 
//   {0.0} });

  
  jointIndex = ctl_.robot().jointIndexByName(jointName);
 

  postureTask = ctl_.getPostureTask(ctl_.robot().name()); 
  postureTask->stiffness(200);
 
 

}

bool SlowDance_Initial::run(mc_control::fsm::Controller & ctl_)
{

  // auto p1 = desired_pos_vector[index_];
  // //auto p2 = desired_pos_vector[index_+1];
  // mc_rtc::log::info("index = {}", index_);
  // mc_rtc::log::info("iter = {}", iter_);

  // for(int i=0; i < p1.size(); i++)
  // {
  //   //double ratio = (iter_ == 0) ? 0 : (double)(iter_%t_) / t_;
  //   double ratio = (iter_ == 0) ? 0 : (double)(iter_ / t_);
  //   mc_rtc::log::info("ratio={}", ratio);
  //   //p1[i][0] += (p2[i][0]-p1[i][0])*ratio;
  //   // shake p1[i][0] += 0.1 * sin(ratio);
  // }

  // postureTask->posture(p1);

  // if(iter_ == 0 || iter_ % t_ == 0)
  // {
  //     //index_++;  en off si shake
  //     index_++;
  // }
  // iter_++;

  // auto & ctl = static_cast<SlowDance &>(ctl_);
  // output("OK");

  // // if(index_ == desired_pos_vector.size()-2)
  // //   return true;
  // // else
  // //   return false;


 //postureTask->posture(desired_pos_vector[index_]);
  
  
  auto neck_posture = ctl_.robot().mbc().q;
  //auto neck_posture = ctl_.robot().jointIndexByName(jointName);
  
  // neck_posture[jointIndex][0] = 0;
  // neck_posture[jointIndex][0] = 0.33;
  // neck_posture[jointIndex][0] = 0.43;
  // neck_posture[jointIndex][0] = -0.33;

  neck_posture.push_back({
  {0.0}, 
  {0.33}, 
  {0.0}, 
  {0.33}, 
  {0.0}, 
  {0.33}, 
  {0.0}, 
  {0.33},  
  {0.0}
  
  });
  
  // auto p1 = neck_posture[jointIndex][index_]; 
  // auto p2 = neck_posture[jointIndex][index_+1];
  mc_rtc::log::info("value = {}", neck_posture[jointIndex][index_]);
  mc_rtc::log::info("iter = {}", iter_);


 for(int i=0; i < neck_posture.size(); i++)
  {
    
    double ratio = (iter_ == 0) ? 0 : (double)(iter_ /t_);
    mc_rtc::log::info("ratio={}", ratio);
    
    neck_posture[jointIndex][0] += 0.2 * sin(ratio);
  }

  //postureTask->target({{jointName, ctl_.robot().q()[jointIndex]}});

  postureTask->posture(neck_posture);
  iter_++;

  auto & ctl = static_cast<SlowDance &>(ctl_);
  output("OK");

  //  if(index_ == desired_pos_vector.size()-2)
  //    return true;
  //  else
  //   return false;



}

void SlowDance_Initial::teardown(mc_control::fsm::Controller & ctl_)
{
  auto & ctl = static_cast<SlowDance &>(ctl_);
}

EXPORT_SINGLE_STATE("SlowDance_Initial", SlowDance_Initial)
